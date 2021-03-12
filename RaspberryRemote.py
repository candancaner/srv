from flask import Flask, request, jsonify
import concurrent.futures as cf
import json
import remotelab
import time
import queue

q_param = queue.LifoQueue()
q_state = queue.Queue()

app = Flask(__name__)


executor = cf.ThreadPoolExecutor(max_workers = 1)

state = {'run': False, 'type':'DEV', 'p':0, 'i':0, 'd':0, 'setpoint':0, 'sensor':0} #Initial value of state
param = {'run': False, 'type':'DEV', 'p':0, 'i':0, 'd':0, 'setpoint':0} #Initial value of state

dof = remotelab.OneDOF()
bbt = remotelab.BBT()
bb = remotelab.BB()

def control():
	try:
		global param
		notify = False
		dof.enable = True
		err_prev = 0
		error_sum = 0
		interval = 0.004
		error_sum_x = 0
		error_sum_y = 0
		_filter_size = 10
		bbt_iter = 0
		x_mov_avg_filter = [0 for i in range(_filter_size)]
		y_mov_avg_filter = [0 for i in range(_filter_size)]
		while True:

			try:
				param = q_param.get_nowait() #Get latest parameters
				notify = True

			except:
				pass

			if param['run']:
	#---------------------- CONTROL OPERATIONS -------------------------------

				if param['type'] == '1DOF Helicopter':

					feedforward = 800
					windup_abs = 1000

					setpoint = float(param['parameters']['setpoint'])
					kp = float(param['parameters']['p'])
					ki = float(param['parameters']['i'])
					kd = float(param['parameters']['d'])

					position = dof.get_encoder()
					error = setpoint - position
					error_sum += error

					p = error * kp
					i = (error_sum if -windup_abs <= error_sum <= windup_abs else error_sum/abs(error_sum)*windup_abs) * ki
					d = (error - err_prev) * kd / interval

					output = feedforward + p + i + d
					output = output if 0 <= output else 0
					dof.set_speed(int(output))
					err_prev = error
					param['parameters']['sensor'] = position

				if param['type'] == 'BB':

					feedforward = 500
					windup_abs = 1000

					setpoint = float(param['parameters']['setpoint'])
					kp = float(param['parameters']['p'])
					ki = float(param['parameters']['i'])
					kd = float(param['parameters']['d'])

					position = bb.get_position()
					error = setpoint - position
					error_sum += error

					p = error * kp
					i = (error_sum if -windup_abs <= error_sum <= windup_abs else error_sum/abs(error_sum)*windup_abs) * ki
					d = (error - err_prev) * kd / interval

					output = feedforward + p + i + d
					output = output if 500 <= output else 500
					output = output if 1000 >= output else 1000
					bb.set_servo(int(output))
					err_prev = error
					param['parameters']['sensor'] = position

				if param['type'] == 'BBT':

					if isinstance(err_prev, int):
						err_prev = (0,0)

					feedforwardx = 725
					feedforwardy = 725
					windup_abs = 500
					calibration_x = (0,0)
					calibration_y = (0,0)

					setpointx = float(param['parameters']['x']['setpoint'])
					kpx = float(param['parameters']['x']['p'])
					kix = float(param['parameters']['x']['i'])
					kdx = float(param['parameters']['x']['d'])

					setpointy = float(param['parameters']['y']['setpoint'])
					kpy = float(param['parameters']['y']['p'])
					kiy = float(param['parameters']['y']['i'])
					kdy = float(param['parameters']['y']['d'])

					positionx, positiony = bbt.get_position()
					positionx = 1000 if positionx > 1000 else positionx
					positiony = 1000 if positiony > 1000 else positiony

					positionx += -100
					positiony += 0


					setpointx = (setpointx + 50) * 3 + 250
					setpointy = (setpointy + 60) * 3 + 250

					error = ((setpointx - positionx), (setpointy - positiony))
					error_sum_x += error[0]
					error_sum_y += error[1]

					#PID for X axis
					px = error[0] * kpx
					ix = (error_sum_x if -windup_abs <= error_sum_x <= windup_abs else error_sum_x/abs(error_sum_x)*windup_abs) * kix

					x_mov_avg_filter[bbt_iter] = (error[0] - err_prev[0]) * kdx / interval
					dx = sum(x_mov_avg_filter[0:_filter_size])

					#PID for Y axis
					py = error[1] * kpy
					iy = (error_sum_y if -windup_abs <= error_sum_y <= windup_abs else error_sum_y/abs(error_sum_y)*windup_abs) * kiy

					y_mov_avg_filter[bbt_iter] = (error[1] - err_prev[1]) * kdy / interval
					dy = sum(y_mov_avg_filter[0:_filter_size])

					bbt_iter = (bbt_iter + 1) % _filter_size

					outputx = feedforwardx + px + ix + dx
					outputy = feedforwardy + py + iy + dy

					outputx += calibration_x[0] * positionx + calibration_x[1]
					outputy += calibration_y[0] * positiony + calibration_y[1]

					#Lower limit for servo
					outputx = 500 if outputx <= 500 else outputx
					outputy = 500 if outputy <= 500 else outputy

					#Upper limit for servo
					outputx = 1000 if outputx >= 1000 else outputx
					outputy = 1000 if outputy >= 1000 else outputy

					####
					positionx = (positionx - 250) / 3 - 50
					positiony = (positiony - 250) / 3 - 60
					bbt.set_servo(int(outputx), int(outputy))
					err_prev = error
					param['parameters']['sensor'] = (positionx, positiony)



	#------------------ END OF CONTROL OPERATIONS ----------------------------

			if notify:
					q_state.put(param)
					notify = False

			if not param['run']:
				if param['type'] == '1DOF Helicopter':
					dof.set_speed(0)
				if param['type'] == 'BB':
					dof.set_servo(750)
				if param['type'] == 'BBT':
					bbt.set_servo(750, 750)
				return

			time.sleep(interval) #Control loop period. !!!! DO NOT USE ANOTHER BLOCKER
	except:
		pass

@app.route('/params', methods=['POST', 'GET'])
def params():
	global state
	r = request.get_json() #Get request body
    
    if request.method == 'POST':        
    	q_param.put(r) #Put param into lifo queue
        #Process request and start control loop if necessary
        if r['run'] and not state['run']:
            #Clear parameter queue at first run
            q_param.queue.clear()
            q_param.put(r) #Put param into lifo queue after clear
            executor.submit(control) #Start control loop

	try:
		state = q_state.get(block=True, timeout=0.1) #Get control state
	except:
		pass
    if request.method == 'GET': pass
        # return jsonify({
        # 'sensor': state['parameters'][sensor]
        # })
	return jsonify(state)

if __name__ == "__main__":
	app.run(debug=False, port=8080)
