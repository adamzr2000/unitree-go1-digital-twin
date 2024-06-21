import threading
import queue
import time
import uuid
import signal
import sys, os
#import shell_wrapper as sw
import yaml
from kubernetes import client, config
from subprocess import run

from flask import Flask, request, jsonify

# teszteles
# curl -F file=@demo_nsd.yml http://localhost:5000/iml/yaml/deploy
# curl -F file=@demo_elte_nsd.yml http://localhost:5000/iml/yaml/deploy
# curl -X DELETE http://localhost:5000/iml/yaml/deploy/5

app = Flask(__name__)

DEPLOY_FOLDER = "deployes"
UPLOAD_FOLDER = 'files'
SWITCH_HOST = '10.5.15.2'
POLICY_CONF_PATH = '/home/admin/ppv/policy.conf'
TRIGGER_PATH = '/home/admin/ppv/trigger.sh'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

deployID = 1
kubectl_cmd = ['kubectl']
if len(sys.argv)>1 and sys.argv[1] == 'ELTE':
  kubectl_cmd = ['microk8s','kubectl']


if not os.path.exists(UPLOAD_FOLDER):
  # Create the directory
    os.makedirs(UPLOAD_FOLDER)

if not os.path.exists(DEPLOY_FOLDER):
  # Create the directory
    os.makedirs(DEPLOY_FOLDER)

# init deploy id
while os.path.exists(DEPLOY_FOLDER+"/"+'deploy-'+str(deployID)+'.yml'):
  deployID += 1

'''
def response_generator(requestQueue, responseData, exit_signal):
  # sh = sw.setupConnection()
  while not exit_signal.is_set():
    if not requestQueue.empty():
      request, token = requestQueue.get()
      print("Got request:", request)
      response = sw.processRequest(request[0], request[1])
      if response == None:
        response = "Can't interpret request"
      responseData[token] = response
      print("response:", response)
    time.sleep(0.1)
  sw.teardownConnection(sh)


requestQueue = queue.Queue()
responseData = {}

exit_signal = threading.Event()	 # Define exit signal event
response_thread = threading.Thread(
  target=response_generator, args=(requestQueue, responseData, exit_signal)
)
response_thread.daemon = True
response_thread.start()
'''
# Dictionary to store tokens and corresponding response data


def getForomNFStore(appId):
	store = {
		'ros-master':'services/ros-master.yml', 
		'go1-base':'services/go1-base.yml', 
		'gesture-control':'services/gesture-control.yml',
		'lidar-drivers':'services/lidar-drivers.yml',
		'go1-navigation':'services/go1-navigation.yml', 
		'rviz-vnc':'services/rviz-vnc.yml', 
		'digital-replica':'services/digital-replica.yml',
		'influxdb':'services/influxdb.yml',
		'grafana':'services/grafana.yml',
		'go1-monitoring-edge':'services/go1-monitoring-edge.yml',
		'go1-monitoring-robot':'services/go1-monitoring-robot.yml'
	}
	return store[appId]
def getFileName(id):
	return DEPLOY_FOLDER+"/"+'deploy-'+str(id)+'.yml'

def createKubernestYaml(appIds):
  global deployID
  _depID = deployID
  fileName = getFileName(_depID)
  deployID += 1
  with open(fileName, 'w') as outfile:
    for apps in appIds:
      with open(getForomNFStore(apps), "r") as infile:
        outfile.write(infile.read())
        outfile.write("\n")
  return (_depID,fileName)

def deployYaml(fileName):
  # kubectl apply -f digital-twin-service.yml
  # result = run(['microk8s','kubectl', 'apply', '-f', fileName])
  return run(kubectl_cmd + ['apply', '-f', fileName], capture_output = True, text = True)
# body = yaml.safe_load("deploy.yml")
  # config.load_kube_config()
  # apps_api = client.AppsV1Api()
  # apps_api.create_namespaced_deployment(body=body, namespace="default")

@app.route("/iml/yaml/deploy/<id>", methods=["DELETE"])
def deleteDeployment(id):
  result = run(kubectl_cmd+['delete', '-f', getFileName(id)], capture_output = True, text = True)
  print(result.stderr)
  if result.stderr:
    return jsonify({"response": result.stderr}), 500
  else:
    return jsonify({"response": "Succesfull delete the deployment with id: "+str(id)}), 200

@app.route("/iml/yaml/deploy", methods=["POST"])
def deploy_yaml():
  # yamlFile = secure_filename(file.filename)

  # content_type = request.headers.get('Content-Type')
  file = request.files['file']
  file.save(os.path.join(app.config['UPLOAD_FOLDER'], "uploaded.yml"))

  yamlFile = app.config['UPLOAD_FOLDER']+"/uploaded.yml"

  with open(yamlFile, 'r') as file:
    try:
      yamlData = yaml.safe_load(file)
      # print("-------------")
      # print(yamlData)
      appList = []
      for apps in yamlData['lnsd']['ns']['application-functions']:
        appList.append(apps['af-id'])

      depID, yamlName = createKubernestYaml(appList)
      result = deployYaml(yamlName)
      if result.stderr:
        response = ("FAILED: "+result.stderr,500)
      else:
        response = ("{Deployed: "+yamlData['lnsd']['ns']['name']+", id:"+str(depID)+"}",200)
    except yaml.parser.ParserError:
      response = ("Faild to parse",500)
  return jsonify({"response": response[0]}), response[1]

def idxToV(x, minV, maxV, w):
  if x < minV:
    return 1023
  elif x > maxV:
    return 0
  else:
    return int(max((1023 - (x - minV) * 1/w), 0))

def createPolicyConf(file):
  data = yaml.safe_load(file)
  policyPath = os.path.join(app.config['UPLOAD_FOLDER'], "policy.conf")
  with open(policyPath, 'w') as f:
    f.write(str(data['policy']['id']) + '\n')
    for x in range(1024):
      f.write(str(idxToV(
        x,
        data['policy']['min-bw'],
        data['policy']['max-bw'],
        data['policy']['elasticity'])) + '\n')
    for u in data['policy']['assigned-user-ids']:
      f.write(str(u) + '\n')
  return [data['policy']['id'], policyPath]

def deployPolicyConf(confPath):
  result = run(['scp', confPath, f'{SWITCH_HOST}:{POLICY_CONF_PATH}'], capture_output = True, text = True)
  if result.stderr:
    raise RuntimeError(result.stderr)
  result = run(['ssh', '-t', SWITCH_HOST, TRIGGER_PATH], capture_output = True, text = True)
  if result.stderr:
    raise RuntimeError(result.stderr)

@app.route("/iml/update-policy", methods=["POST"])
def updatePolicy():
  file = request.files['file']
  ymlPath = os.path.join(app.config['UPLOAD_FOLDER'], "uploaded-policy.yml")
  file.save(ymlPath)

  with open(ymlPath, 'r') as file:
    try:
      pId, pFile = createPolicyConf(file)
      deployPolicyConf(pFile)
      response = (f"{{Successfully updated policy with id: {pId} }}", 200)

    except yaml.parser.ParserError:
      response = ("Faild to parse", 500)
    except RuntimeError as e:
      response = (f"FAILED: {str(e)}",500)
  return jsonify({"response": response[0]}), response[1]

if __name__ == "__main__":
	app.run(host='0.0.0.0', debug=True)

