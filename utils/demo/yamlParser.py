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
POLICY_CONF_PATH = '/home/netcom/ppv/policy.conf'
TRIGGER_PATH = '/home/netcom/ppv/trigger.sh'
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
      'ros-network':'services/ros-network.yml',    
      'ros-master':'services/ros-master.yml',
      'go1-base':'services/go1-base.yml',
      'gesture-control':'services/gesture-control.yml',
      'lidar-drivers':'services/lidar-drivers.yml',
      'video-streamer':'services/video-streamer.yml',
      'video-receiver':'services/video-receiver.yml',
      'mediamtx':'services/mediamtx.yml',
      'go1-navigation':'services/go1-navigation.yml',
      'go1-navigation-cpu-stress':'services/go1-navigation-cpu-stress.yml',
      'vizanti':'services/vizanti.yml',
      'digital-replica':'services/digital-replica.yml',
      'mas-service-001':'services/mas-pipeline-descriptor.yml'
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

ridx = {
  0 : 0,  0 : 0,  0 : 0,  160 : 1,  200 : 133,  240 : 148,  280 : 161,  320 : 172,  360 : 182,  400 : 190,  440 : 198,  480 : 205,  520 : 212,  560 : 218,  600 : 224,  640 : 229,  680 : 234,  720 : 239,  760 : 243,  800 : 248,  840 : 252,  880 : 255,  920 : 259,  960 : 263,  1000 : 266,  1040 : 269,  1080 : 272,  1120 : 275,  1160 : 278,  1200 : 281,  1240 : 284,  1280 : 286,  1320 : 289,  1360 : 291,  1400 : 294,  1440 : 296,  1480 : 298,  1520 : 300,  1560 : 303,  1600 : 305,  1640 : 307,  1680 : 309,  1720 : 311,  1760 : 313,  1800 : 314,  1840 : 316,  1880 : 318,  1920 : 320,  1960 : 321,  2000 : 323,  2040 : 325,  2080 : 326,  2120 : 328,  2160 : 329,  2200 : 331,  2240 : 332,  2280 : 334,  2320 : 335,  2360 : 337,  2400 : 338,  2440 : 339,  2480 : 341,  2520 : 342,  2560 : 343,  2600 : 345,  2640 : 346,  2680 : 347,  2720 : 348,  2760 : 350,  2800 : 351,  2840 : 352,  2880 : 353,  2920 : 354,  2960 : 355,  3000 : 357,  3040 : 358,  3080 : 359,  3120 : 360,  3160 : 361,  3200 : 362,  3240 : 363,  3280 : 364,  3320 : 365,  3360 : 366,  3400 : 367,  3440 : 368,  3480 : 369,  3520 : 370,  3560 : 371,  3600 : 372,  3680 : 373,  3720 : 374,  3760 : 375,  3800 : 376,  3840 : 377,  3880 : 378,  3920 : 379,  4000 : 380,  4040 : 381,  4080 : 382,  4120 : 383,  4200 : 384,  4240 : 385,  4280 : 386,  4320 : 387,  4400 : 388,  4440 : 389,  4480 : 390,  4560 : 391,  4600 : 392,  4680 : 393,  4720 : 394,  4760 : 395,  4840 : 396,  4880 : 397,  4960 : 398,  5000 : 399,  5080 : 400,  5120 : 401,  5200 : 402,  5240 : 403,  5320 : 404,  5400 : 405,  5440 : 406,  5520 : 407,  5600 : 408,  5640 : 409,  5720 : 410,  5800 : 411,  5880 : 412,  5920 : 413,  6000 : 414,  6080 : 415,  6160 : 416,  6240 : 417,  6320 : 418,  6400 : 419,  6440 : 420,  6520 : 421,  6600 : 422,  6680 : 423,  6760 : 424,  6880 : 425,  6960 : 426,  7040 : 427,  7120 : 428,  7200 : 429,  7280 : 430,  7360 : 431,  7480 : 432,  7560 : 433,  7640 : 434,  7760 : 435,  7840 : 436,  7920 : 437,  8040 : 438,  8120 : 439,  8240 : 440,  8320 : 441,  8440 : 442,  8520 : 443,  8640 : 444,  8720 : 445,  8840 : 446,  8960 : 447,  9080 : 448,  9160 : 449,  9280 : 450,  9400 : 451,  9520 : 452,  9640 : 453,  9760 : 454,  9880 : 455,  10000 : 456,  10120 : 457,  10240 : 458,  10360 : 459,  10480 : 460,  10600 : 461,  10720 : 462,  10880 : 463,  11000 : 464,  11120 : 465,  11280 : 466,  11400 : 467,  11560 : 468,  11680 : 469,  11840 : 470,  11960 : 471,  12120 : 472,  12280 : 473,  12400 : 474,  12560 : 475,  12720 : 476,  12880 : 477,  13040 : 478,  13200 : 479,  13360 : 480,  13520 : 481,  13680 : 482,  13840 : 483,  14000 : 484,  14160 : 485,  14360 : 486,  14520 : 487,  14720 : 488,  14880 : 489,  15080 : 490,  15240 : 491,  15440 : 492,  15640 : 493,  15800 : 494,  16000 : 495,  16200 : 496,  16400 : 497,  16600 : 498,  16800 : 499,  17000 : 500,  17200 : 501,  17440 : 502,  17640 : 503,  17840 : 504,  18080 : 505,  18280 : 506,  18520 : 507,  18720 : 508,  18960 : 509,  19200 : 510,  19440 : 511,  19680 : 512,  19920 : 513,  20160 : 514,  20400 : 515,  20640 : 516,  20880 : 517,  21160 : 518,  21400 : 519,  21680 : 520,  21920 : 521,  22200 : 522,  22480 : 523,  22760 : 524,  23000 : 525,  23280 : 526,  23600 : 527,  23880 : 528,  24160 : 529,  24440 : 530,  24760 : 531,  25040 : 532,  25360 : 533,  25680 : 534,  26000 : 535,  26320 : 536,  26640 : 537,  26960 : 538,  27280 : 539,  27600 : 540,  27960 : 541,  28280 : 542,  28640 : 543,  28960 : 544,  29320 : 545,  29680 : 546,  30040 : 547,  30400 : 548,  30800 : 549,  31160 : 550,  31560 : 551,  31920 : 552,  32320 : 553,  32720 : 554,  33120 : 555,  33520 : 556,  33920 : 557,  34320 : 558,  34760 : 559,  35160 : 560,  35600 : 561,  36040 : 562,  36480 : 563,  36920 : 564,  37360 : 565,  37840 : 566,  38280 : 567,  38760 : 568,  39240 : 569,  39720 : 570,  40200 : 571,  40680 : 572,  41200 : 573,  41680 : 574,  42200 : 575,  42720 : 576,  43240 : 577,  43760 : 578,  44280 : 579,  44840 : 580,  45360 : 581,  45920 : 582,  46480 : 583,  47040 : 584,  47640 : 585,  48200 : 586,  48800 : 587,  49400 : 588,  50000 : 589,  50600 : 590,  51240 : 591,  51840 : 592,  52480 : 593,  53120 : 594,  53760 : 595,  54440 : 596,  55080 : 597,  55760 : 598,  56440 : 599,  57120 : 600,  57840 : 601,  58520 : 602,  59240 : 603,  59960 : 604,  60680 : 605,  61440 : 606,  62200 : 607,  62960 : 608,  63720 : 609,  64480 : 610,  65280 : 611,  66080 : 612,  66880 : 613,  67680 : 614,  68520 : 615,  69360 : 616,  70200 : 617,  71040 : 618,  71920 : 619,  72800 : 620,  73680 : 621,  74600 : 622,  75480 : 623,  76440 : 624,  77360 : 625,  78280 : 626,  79240 : 627,  80200 : 628,  81200 : 629,  82200 : 630,  83200 : 631,  84200 : 632,  85240 : 633,  86280 : 634,  87320 : 635,  88400 : 636,  89480 : 637,  90560 : 638,  91680 : 639,  92760 : 640,  93920 : 641,  95040 : 642,  96200 : 643,  97400 : 644,  98560 : 645,  99800 : 646,  101000 : 647,  102240 : 648,  103480 : 649,  104720 : 650,  106000 : 651,  107320 : 652,  108600 : 653,  109960 : 654,  111280 : 655,  112640 : 656,  114000 : 657,  115400 : 658,  116800 : 659,  118240 : 660,  119680 : 661,  121160 : 662,  122600 : 663,  124120 : 664,  125640 : 665,  127160 : 666,  128720 : 667,  130280 : 668,  131880 : 669,  133480 : 670,  135120 : 671,  136760 : 672,  138440 : 673,  140120 : 674,  141840 : 675,  143560 : 676,  145320 : 677,  147080 : 678,  148880 : 679,  150680 : 680,  152520 : 681,  154400 : 682,  156280 : 683,  158160 : 684,  160120 : 685,  162040 : 686,  164040 : 687,  166040 : 688,  168080 : 689,  170120 : 690,  172200 : 691,  174280 : 692,  176400 : 693,  178560 : 694,  180760 : 695,  182960 : 696,  185200 : 697,  187440 : 698,  189720 : 699,  192040 : 700,  194400 : 701,  196760 : 702,  199160 : 703,  201600 : 704,  204040 : 705,  206520 : 706,  209040 : 707,  211600 : 708,  214200 : 709,  216800 : 710,  219440 : 711,  222120 : 712,  224840 : 713,  227560 : 714,  230360 : 715,  233160 : 716,  236000 : 717,  238880 : 718,  241800 : 719,  244760 : 720,  247720 : 721,  250760 : 722,  253800 : 723,  256920 : 724,  260040 : 725,  263200 : 726,  266440 : 727,  269680 : 728,  272960 : 729,  276280 : 730,  279680 : 731,  283080 : 732,  286520 : 733,  290040 : 734,  293560 : 735,  297160 : 736,  300760 : 737,  304440 : 738,  308160 : 739,  311920 : 740,  315720 : 741,  319560 : 742,  323480 : 743,  327400 : 744,  331400 : 745,  335440 : 746,  339560 : 747,  343680 : 748,  347880 : 749,  352120 : 750,  356440 : 751,  360760 : 752,  365160 : 753,  369640 : 754,  374120 : 755,  378720 : 756,  383320 : 757,  388000 : 758,  392720 : 759,  397520 : 760,  402360 : 761,  407280 : 762,  412240 : 763,  417280 : 764,  422360 : 765,  427520 : 766,  432720 : 767,  438000 : 768,  443360 : 769,  448760 : 770,  454240 : 771,  459800 : 772,  465400 : 773,  471080 : 774,  476840 : 775,  482640 : 776,  488520 : 777,  494480 : 778,  500520 : 779,  506640 : 780,  512800 : 781,  519080 : 782,  525400 : 783,  531800 : 784,  538280 : 785,  544880 : 786,  551520 : 787,  558240 : 788,  565040 : 789,  571960 : 790,  578920 : 791,  586000 : 792,  593120 : 793,  600360 : 794,  607680 : 795,  615120 : 796,  622600 : 797,  630200 : 798,  637880 : 799,  645680 : 800,  653560 : 801,  661520 : 802,  669600 : 803,  677760 : 804,  686040 : 805,  694400 : 806,  702880 : 807,  711440 : 808,  720120 : 809,  728920 : 810,  737800 : 811,  746800 : 812,  755920 : 813,  765160 : 814,  774480 : 815,  783920 : 816,  793480 : 817,  803160 : 818,  812960 : 819,  822880 : 820,  832920 : 821,  843080 : 822,  853360 : 823,  863800 : 824,  874320 : 825,  885000 : 826,  895800 : 827,  906720 : 828,  917800 : 829,  929000 : 830,  940320 : 831,  951800 : 832,  963400 : 833,  975160 : 834,  987040 : 835,  999080 : 836,  1011280 : 837,  1023600 : 838,  1036120 : 839,  1048760 : 840,  1061560 : 841,  1074480 : 842,  1087600 : 843,  1100880 : 844,  1114320 : 845,  1127880 : 846,  1141640 : 847,  1155600 : 848,  1169680 : 849,  1183960 : 850,  1198400 : 851,  1213000 : 852,  1227800 : 853,  1242800 : 854,  1257960 : 855,  1273320 : 856,  1288840 : 857,  1304560 : 858,  1320480 : 859,  1336600 : 860,  1352880 : 861,  1369400 : 862,  1386120 : 863,  1403040 : 864,  1420120 : 865,  1437480 : 866,  1455000 : 867,  1472760 : 868,  1490720 : 869,  1508920 : 870,  1527320 : 871,  1545960 : 872,  1564800 : 873,  1583920 : 874,  1603240 : 875,  1622800 : 876,  1642600 : 877,  1662640 : 878,  1682920 : 879,  1703440 : 880,  1724240 : 881,  1745280 : 882,  1766560 : 883,  1788120 : 884,  1809920 : 885,  1832000 : 886,  1854360 : 887,  1876960 : 888,  1899880 : 889,  1923040 : 890,  1946520 : 891,  1970280 : 892,  1994320 : 893,  2018640 : 894,  2043280 : 895,  2068200 : 896,  2093440 : 897,  2118960 : 898,  2144800 : 899,  2171000 : 900,  2197480 : 901,  2224280 : 902,  2251400 : 903,  2278880 : 904,  2306680 : 905,  2334840 : 906,  2363320 : 907,  2392160 : 908,  2421320 : 909,  2450880 : 910,  2480760 : 911,  2511040 : 912,  2541680 : 913,  2572680 : 914,  2604080 : 915,  2635840 : 916,  2668000 : 917,  2700560 : 918,  2733520 : 919,  2766840 : 920,  2800600 : 921,  2834760 : 922,  2869360 : 923,  2904360 : 924,  2939800 : 925,  2975680 : 926,  3011960 : 927,  3048720 : 928,  3085920 : 929,  3123560 : 930,  3161680 : 931,  3200240 : 932,  3239280 : 933,  3278800 : 934,  3318800 : 935,  3359320 : 936,  3400280 : 937,  3441760 : 938,  3483760 : 939,  3526280 : 940,  3569280 : 941,  3612840 : 942,  3656920 : 943,  3701520 : 944,  3746680 : 945,  3792400 : 946,  3838680 : 947,  3885520 : 948,  3932920 : 949,  3980880 : 950,  4029480 : 951,  4078640 : 952,  4128400 : 953,  4178760 : 954,  4229720 : 955,  4281320 : 956,  4333560 : 957,  4386440 : 958,  4439960 : 959,  4494120 : 960,  4548960 : 961,  4604440 : 962,  4660640 : 963,  4717480 : 964,  4775040 : 965,  4833320 : 966,  4892280 : 967,  4951960 : 968,  5012360 : 969,  5073520 : 970,  5135440 : 971,  5198080 : 972,  5261520 : 973,  5325680 : 974,  5390680 : 975,  5456440 : 976,  5523000 : 977,  5590400 : 978,  5658600 : 979,  5727640 : 980,  5797520 : 981,  5868240 : 982,  5939840 : 983,  6012320 : 984,  6085640 : 985,  6159920 : 986,  6235040 : 987,  6311120 : 988,  6388120 : 989,  6466080 : 990,  6544960 : 991,  6624800 : 992,  6705640 : 993,  6787440 : 994,  6870240 : 995,  6954080 : 996,  7038920 : 997,  7124800 : 998,  7211720 : 999,  7299680 : 1000,  7388760 : 1001,  7478920 : 1002,  7570160 : 1003,  7662520 : 1004,  7756000 : 1005,  7850640 : 1006,  7946400 : 1007,  8043360 : 1008,  8141480 : 1009,  8240800 : 1010,  8341360 : 1011,  8443120 : 1012,  8546120 : 1013,  8650400 : 1014,  8755920 : 1015,  8862760 : 1016,  8970880 : 1017,  9080320 : 1018,  9191120 : 1019,  9303240 : 1020,  9416760 : 1021,  9531640 : 1022,  9647920 : 1023}

def KbpsToRidx(rateKbps):
  global ridx
  index = 0
  for r in ridx.keys(): # Assuming it is sorted
      if rateKbps>r:
          index = ridx[r]
      else:
          break
  return index

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
  minbw = KbpsToRidx(data['policy']['min-bw'])
  maxbw = KbpsToRidx(data['policy']['max-bw'])
  with open(policyPath, 'w') as f:
    f.write(str(data['policy']['id']) + '\n')
    for x in range(1024):
      f.write(str(idxToV(
        x,
        minbw,
        maxbw,
        data['policy']['elasticity'])) + '\n')
    for u in data['policy']['assigned-user-ids']:
      f.write(str(u) + '\n')
  return [data['policy']['id'], policyPath]

def deployPolicyConf(confPath):
  result = run(['scp', confPath, f'{SWITCH_HOST}:{POLICY_CONF_PATH}'], capture_output = True, text = True)
  if result.stderr:
    raise RuntimeError(result.stderr)
  result = run(['ssh', '-t', SWITCH_HOST, TRIGGER_PATH], capture_output = True, text = True)
  if result.returncode != 0:
    raise RuntimeError(result.stderr)

def loadPolicyFile(filename):
  with open(filename, 'r') as file:
    try:
      pId, pFile = createPolicyConf(file)
      deployPolicyConf(pFile)
      response = (f"{{Successfully updated policy with id: {pId} }}", 200)
    except yaml.parser.ParserError:
      response = ("Faild to parse", 500)
    except RuntimeError as e:
      response = (f"FAILED: {str(e)}",500)
    return response

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

@app.route("/iml/sla-notification", methods=["POST"])
def SlaNotification():
  print("Throughput SLA violation.")
  print("Start policy reconfiguration and notify SMO.")
  loadPolicyFile("demo_pol_3_perf_isolation_different_ns-slice_1-robot.yml")
#  time.sleep(1)
  resp = loadPolicyFile("demo_pol_3_perf_isolation_different_ns-slice_1-bg.yml")
#  time.sleep(1)
  resp = loadPolicyFile("demo_pol_3NEW_perf_isolation_same_pol_ns-slice_2.yml")
  print(resp[0])
  response = (f"{{IML has been notified.}}", 200)
  return jsonify({"response": response[0]}), response[1]

  
if __name__ == "__main__":
  app.run(host='0.0.0.0', debug=True)
