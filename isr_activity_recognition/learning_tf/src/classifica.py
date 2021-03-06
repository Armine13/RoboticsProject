#!/usr/bin/env python  
import roslib
import rospy
from std_msgs.msg import String
import math
import tf
import geometry_msgs.msg
import numpy as np
import hcluster as mat
import pickle
import matplotlib.pyplot as plt
from sklearn import *
from scipy import ndimage, signal
import os.path
import time
from itertools import islice
import scipy.io as sio
from subprocess import Popen, PIPE
from scipy import linalg as lina

conta_class = np.zeros((8))
actividades =['Walking', 'Standing still', 'Working on computer', 'Talking on the phone','Running','Jumping','Falling','Sitting down'] # Activities

# Callback for reaction of the recognized activity "Falling"
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    
    if data.data == "yes" or data.data == "yes please" or data.data == "help" or data.data == "help me" or data.data == "please":
        os.system("rosrun sound_play say.py 'calling a doctor'")        

# Function that listens the answer of the person to a question    
def resposta():
    #rospy.init_node('classifica', anonymous=True)

    rospy.Subscriber('recognizer/output', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



#compute the weights trough entorpy - uncertainty measure to find
#the cnfidence level of a classifer
#input:  classifiers n x m matrix: n = probabilities of the m_th classifier type
#output: weights 1xm
def conf_class(classifiers):
    offset = 0.9;
           
    for j in range(0,np.shape(classifiers, 1)):  
        #avoiding extremes
        classifiers[classifiers<=0.0001]=0.001;
        classifiers[classifiers>0.989]=0.99;               
        logar = np.log10(classifiers[:,j]);
        prob =  classifiers[:,j];
        h = prob*logar;
        w[j] = -np.sum(h);
    
    summ = sum(w);
    for j in range(0,np.shape(w, 1)):       
        w[j] = 1 - (w(j)/ summ);
           
    #avoiding extremes
    w[w>0.998] = offset;
    w[w<=0.0001] = 1-offset;             
           
    summ = sum(w);
    for j in range(0,np.shape(w, 1)):     
        w[j] = w[j]/summ; 

    return w

def weight_update(prior_weights, post3rd):
         
    # computing the weights distribution inversely proportional to the entropy given the 3rd order Markov property 
    #       i.e., given the posteriors: P(C_x|C_t-i), i=1,2,3, x=t, t-1, t-2
    w_t = conf_class(post3rd);
         
    # convert the current vector of weights into a diagonal matrix for multiplication with the prior
    D_wt = np.diag(w_t);
    weights = np.diag(prior_weights) * D_wt;
    summ = np.sum(np.diag(weights));
    weights = np.diag(weights) / summ; # normalization / distribution
 
    return weights

#adjust	the convergence, avoid the class reaching 1 or 0 and keeping it, allowing state transitions in framexframe classification	   
def adjust_prob_conv(dbmmcl, num_cl):
    yn = 0;
    j=0;
    val=0.65;
    for c in range(0,num_cl):
        if (dbmmcl[j, c]>val):
	    yn = 1;
	    #resta = 1 - (dbmmcl(j, c)+(0.3/num_cl));
            resta = 1 - val;
	    r = resta/(num_cl-1);
	    dbmmcl[j, :] = r;
	    dbmmcl[j, c] = val;
        elif (dbmmcl[j, c] < 0.01):
	    yn = 1;
	    dbmmcl[j, c] = 0.01;   
		
	if (yn):
	    sumc = np.sum(dbmmcl[j, :]);
	    dbmmcl[j, :] = dbmmcl[j, :] / sumc;                 
	
    return dbmmcl 	


############################### DBMM #################################

def dbmm(class_nb, class_svm, weights, labels):

    dbmmcl = np.array([[]]); dbmm_nop= np.array([[]]); confusion = np.array([[]]); prec = np.array([[]]); rec = np.array([[]]);
	# %normalization of the features
	# class_feat = normalize_feat(class_feat);
	
    # ensure that the label vector is within dimension frames x 1
    #if (labels.shape[1] > labels.shape[0]):
     #   labels = labels.T;
   
  
    # number of frames
    frames = class_nb.shape[0];
    # number of classifiers
    num_cl = 2
    size_class = class_nb.shape[1];

    #for weights update to control the short-time memory (5rd order Markov property)
    short_time_wup = 0;
    
    #if there is a label vector
    '''if (labels.shape[0] > 1):ls
	
        # get the number of classes from labels
	size_class = 8;		
        
	# prepare a variable with the boundaries of the class labels
	lbl_bound = np.array([[]])
        classes_te = np.unique(labels);
	for c in range(0,size_class):
	    ini = np.where(labels==classes_te[c])[0][0]
	    fim = np.where(labels==classes_te[c])[0][0]
            fim = fim[0][np.size(fim)-1]
	    lbl_bound = np.r_[lbl_bound, np.c_[ini, fim]]'''

    # verify if there is any NaN value for each base classifier
    '''for k in range(0,frames):
        if (np.isnan(class_nb[k, :])):
	    class_nb[k, :] = class_nb[k-1, :];
        if (np.isnan(class_svm[k, :])):
	    class_svm[k, :] = class_svm[k-1, :];'''

    # initially, uniform prior
    dbmmcl = np.zeros((frames,size_class))

    prior = 1.0/size_class;  

    first_result_nb = weights[0]*class_nb[0,:];
    first_result_svm = weights[1]*class_svm[0,:];

    dbmmcl[0,:] = first_result_nb + first_result_svm
    dbmm_nop = dbmmcl;
    dbmmcl[0,:] = dbmmcl[0,:]*prior;
    dbmmcl[0,:] = dbmmcl[0,:] / np.sum(dbmmcl[0,:]);   

    ######## DBMM - fusion
    for j in range(1,frames):
        sumw=np.sum(weights);
        weights = weights/sumw;

        # DBMM weighted sum
	w_classif_nb = weights[0]*class_nb[j,:];
	w_classif_svm = weights[1]*class_svm[j,:];	
	
	pond = w_classif_nb + w_classif_svm 
	dbmm_nop = pond;
	  
	#transition probability using prior P(C_t | C_t-1) - dynamic probabilistic loop - 1st order Markov property
        dbmmcl[j, :] =  dbmmcl[j-1, :]*pond;
	#normalization
        sc = np.sum(dbmmcl[j, :]);
	dbmmcl[j, :] = dbmmcl[j, :]/sc;
        dbmmcl[j, :] = dbmmcl[j, :];

	short_time_wup = short_time_wup + 1;
      
        dbmm_cl = adjust_prob_conv(np.array([dbmmcl[j, :]]), dbmmcl.shape[1]);
	dbmmcl[j, :] = dbmm_cl;

    return [dbmmcl, dbmm_nop]


################## Moving Average Filter ######################### 
def moving_average(a, n) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


#################### Log-Cov Function ####################### 
def apply_log_vect(M):

    d = M.shape[0]
    n = d*(d+1)/2
    V = np.zeros((n,1))
    offset = 0.001*np.eye(d,d)
    true_mat = np.ones((d,d))
    true_mat = true_mat.astype(np.int64)
    in_triu = np.triu(true_mat)
    logM = np.real(lina.logm(M+offset))
    V = logM[in_triu==1]
    V = np.array([V])
    return V

################### Features Extraction Function ############## 
def feature_extraction_torso_camera(input_torso, input_camera):

    numero_juntas = 15;   # number of joints
    frame_rate=1/30.0;    # frame rate
    window = 10;          # temporal window

    x = input_torso[:,0::6]
    y = input_torso[:,1::6]
    z = input_torso[:,2::6]

    # Guarantees that the number of frames is the same for torso and camera features
    if input_torso.size < input_camera.size:
        [m, n] = input_torso.shape
    else:
        [m, n] = input_camera.shape
    
    ## Log-Cov of distances between every joints relative to the torso

    distancias = np.zeros((numero_juntas,numero_juntas))
    distancias_total=np.array([[]])
    
    for frame in range(0,m):
        for i in range(0,15):
            for j in range(0,15):
                distancias[i,j]= mat.pdist([[x[frame,i], y[frame,i], z[frame,i]], [x[frame,j], y[frame,j], z[frame,j]]])
       
        distlower = np.tril(distancias)
        distupper = np.triu(distancias)
        distancias_final = distlower[1:, :] + distupper[0:-1,:] # elimination of null diagonal
        
        cov_distancias = np.cov(distancias_final.T)
        #cov_distancias_final = np.triu(cov_distancias)
        #aux = np.reshape(cov_distancias_final.T,(1,15*15)).copy() 
        #aux=np.array([aux[aux!=0]])
        
        aux = apply_log_vect(cov_distancias)

        #aux2 = np.reshape(distlower.T,(1,np.size(distlower))).copy()
        #aux2 = np.array([aux2[aux2!=0]])
        
        distancias_total = np.concatenate([distancias_total, aux]) if distancias_total.size else aux
        
          
    ## Distances between every joints and torso
    distancias=np.zeros((m,numero_juntas))
    
    for frame in range(0,m):
        for i in range(0,15):
            distancias[frame,i]= mat.pdist([[x[frame,i], y[frame,i], z[frame,i]], [x[frame,3], y[frame,3], z[frame,3]]])

    distancias_ao_torso = distancias 
        
    ## Absolute velocities

    velocidades=np.zeros((m,numero_juntas))

    for frame in range(0,m):
    
        if frame == 0:
            anterior = frame;
        else:
            anterior = frame-1;
        
        actual = frame;
    
        for i in range(0,15):
            velocidades[frame,i]= (mat.pdist([[x[actual,i], y[actual,i], z[actual,i]], [x[anterior,i], y[anterior,i], z[anterior,i]]]))/(frame_rate)
    
    velocidades_total = velocidades
    
    ## Velocities and directions for each dimension {x,y,z}

    vx = np.zeros((m,numero_juntas))
    vy = np.zeros((m,numero_juntas))
    vz = np.zeros((m,numero_juntas))
    dx = np.zeros((m,numero_juntas))
    dy = np.zeros((m,numero_juntas))
    dz = np.zeros((m,numero_juntas))

    for frame in range(0,m):
    
        if frame == 0:
            anterior = frame;
        else:
            anterior = frame-1;
        
        actual = frame
    
        for i in range(0,15):
            dx[frame,i] = x[actual,i]-x[anterior,i]
            dy[frame,i] = y[actual,i]-y[anterior,i]
            dz[frame,i] = z[actual,i]-z[anterior,i]
            vx[frame,i] = dx[frame,i]/(frame_rate);
            vy[frame,i] = dy[frame,i]/(frame_rate);
            vz[frame,i] = dz[frame,i]/(frame_rate);

    velocidade_xyz = np.c_[vx, vy, vz]
    direcao_xyz = np.c_[dx, dy, dz]

    ## Angles of the triangles formed by {shoulders, elbows, hands}, {shoulders, hips, knees} and {hips, knees, feet}
    angulos = np.array([])

    for frame in range(0,m):
    
        ombro_esq_cotovelo_esq = mat.pdist([[x[frame,4-1], y[frame,4-1], z[frame,4-1]], [x[frame,5-1], y[frame,5-1], z[frame,5-1]]]);      # distance between left shoulder and left elbow 
        ombro_esq_mao_esq = mat.pdist([[x[frame,4-1], y[frame,4-1], z[frame,4-1]], [x[frame,12-1], y[frame,12-1], z[frame,12-1]]]);        # distance between left shoulder and left hand 
        mao_esq_cotovelo_esq = mat.pdist([[x[frame,12-1], y[frame,12-1], z[frame,12-1]], [x[frame,5-1], y[frame,5-1], z[frame,5-1]]]);     # distance between left hand and left elbow  
        angulo_esq1 = np.arccos((ombro_esq_cotovelo_esq**2 + mao_esq_cotovelo_esq**2 - ombro_esq_mao_esq**2)/(2*ombro_esq_cotovelo_esq*mao_esq_cotovelo_esq)); # angle
     
        ombro_dir_cotovelo_dir = mat.pdist([[x[frame,6-1], y[frame,6-1], z[frame,6-1]], [x[frame,7-1], y[frame,7-1], z[frame,7-1]]]);      # distance between right shoulder and right elbow 
        ombro_dir_mao_dir = mat.pdist([[x[frame,6-1], y[frame,6-1], z[frame,6-1]], [x[frame,13-1], y[frame,13-1], z[frame,13-1]]]);        # distance between right shoulder and right hand 
        mao_dir_cotovelo_dir = mat.pdist([[x[frame,13-1], y[frame,13-1], z[frame,13-1]], [x[frame,7-1], y[frame,7-1], z[frame,7-1]]]);     # distance between right hand and right elbow  
        angulo_dir1 = np.arccos((ombro_dir_cotovelo_dir**2 + mao_dir_cotovelo_dir**2 - ombro_dir_mao_dir**2)/(2*ombro_dir_cotovelo_dir*mao_dir_cotovelo_dir)); # angle
     
        ombro_esq_anca_esq = mat.pdist([[x[frame,4-1], y[frame,4-1], z[frame,4-1]], [x[frame,8-1], y[frame,8-1], z[frame,8-1]]]);          # distance between left shoulder and left hip 
        ombro_esq_joelho_esq = mat.pdist([[x[frame,4-1], y[frame,4-1], z[frame,4-1]], [x[frame,9-1], y[frame,9-1], z[frame,9-1]]]);        # distance between left shoulder and left knee 
        anca_esq_joelho_esq = mat.pdist([[x[frame,8-1], y[frame,8-1], z[frame,8-1]], [x[frame,9-1], y[frame,9-1], z[frame,9-1]]]);         # distance between left hip and left knee 
        angulo_esq2 = np.arccos((ombro_esq_anca_esq**2 + anca_esq_joelho_esq**2 - ombro_esq_joelho_esq**2)/(2*ombro_esq_anca_esq*anca_esq_joelho_esq));        # angle
     
        ombro_dir_anca_dir = mat.pdist([[x[frame,6-1], y[frame,6-1], z[frame,6-1]], [x[frame,10-1], y[frame,10-1], z[frame,10-1]]]);       # distance between right shoulder and right hip 
        ombro_dir_joelho_dir = mat.pdist([[x[frame,6-1], y[frame,6-1], z[frame,6-1]], [x[frame,11-1], y[frame,11-1], z[frame,11-1]]]);     # distance between right shoulder and right knee 
        anca_dir_joelho_dir = mat.pdist([[x[frame,10-1], y[frame,10-1], z[frame,10-1]], [x[frame,11-1], y[frame,11-1], z[frame,11-1]]]);   # distance between right hip and right knee  
        angulo_dir2 = np.arccos((ombro_dir_anca_dir**2 + anca_dir_joelho_dir**2 - ombro_dir_joelho_dir**2)/(2*ombro_dir_anca_dir*anca_dir_joelho_dir));        # angle
     
        pe_esq_anca_esq = mat.pdist([[x[frame,14-1], y[frame,14-1], z[frame,14-1]], [x[frame,8-1], y[frame,8-1], z[frame,8-1]]]);          # distance between left foot and left hip 
        pe_esq_joelho_esq = mat.pdist([[x[frame,14-1], y[frame,14-1], z[frame,14-1]], [x[frame,9-1], y[frame,9-1], z[frame,9-1]]]);        # distance between left foot and left knee
        anca_esq_joelho_esq = mat.pdist([[x[frame,8-1], y[frame,8-1], z[frame,8-1]], [x[frame,9-1], y[frame,9-1], z[frame,9-1]]]);         # distance between left hip and left knee
        angulo_esq3 = np.arccos((pe_esq_joelho_esq**2 + anca_esq_joelho_esq**2 - pe_esq_anca_esq**2)/(2*pe_esq_joelho_esq*anca_esq_joelho_esq));               # angle

        pe_dir_anca_dir = mat.pdist([[x[frame,15-1], y[frame,15-1], z[frame,15-1]], [x[frame,10-1], y[frame,10-1], z[frame,10-1]]]);       # distance between right foot and right hip
        pe_dir_joelho_dir = mat.pdist([[x[frame,15-1], y[frame,15-1], z[frame,15-1]], [x[frame,11-1], y[frame,11-1], z[frame,11-1]]]);     # distance between right foot and right knee
        anca_dir_joelho_dir = mat.pdist([[x[frame,10-1], y[frame,10-1], z[frame,10-1]], [x[frame,11-1], y[frame,11-1], z[frame,11-1]]]);   # distance between right hip and right knee 
        angulo_dir3 = np.arccos((pe_dir_joelho_dir**2 + anca_dir_joelho_dir**2 - pe_dir_anca_dir**2)/(2*pe_dir_joelho_dir*anca_dir_joelho_dir));               # angle
              
        an = np.c_[angulo_esq1, angulo_dir1, angulo_esq2, angulo_dir2, angulo_esq3, angulo_dir3] 
        angulos = np.r_[angulos, an] if angulos.size else an
        
    ## Angular Difference

    variacao_angulos = np.array([[]])
    for frame in range(0,m):
        if frame == 0:
           anterior = frame;
        else:
            anterior = frame-1;
    
        actual = frame;
    
        dif = np.array([angulos[actual,:] - angulos[anterior,:]])
        variacao_angulos = np.r_[variacao_angulos, dif] if variacao_angulos.size else dif  
   
    ## Variation of all joints relative to the camera in {x,y,z}
    x_camera = input_camera[:,0::6]
    y_camera = input_camera[:,1::6]
    z_camera = input_camera[:,2::6]
    dx_camera = np.zeros((m,numero_juntas));
    dy_camera = np.zeros((m,numero_juntas));
    dz_camera = np.zeros((m,numero_juntas));
    vx_camera = np.zeros((m,numero_juntas));
    vy_camera = np.zeros((m,numero_juntas));
    vz_camera = np.zeros((m,numero_juntas));

    for frame in range(0,m):
        if frame == 0:
            anterior = frame;
        else:
            anterior = frame-1;

        actual = frame;
    
        for i in range(0,15):
            dx_camera[frame,i] = x_camera[actual,i]-x_camera[anterior,i];
            dy_camera[frame,i] = y_camera[actual,i]-y_camera[anterior,i];
            dz_camera[frame,i] = z_camera[actual,i]-z_camera[anterior,i];
            vx_camera[frame,i] = dx_camera[frame,i]/(frame_rate);
            vy_camera[frame,i] = dy_camera[frame,i]/(frame_rate);
            vz_camera[frame,i] = dz_camera[frame,i]/(frame_rate);

    variacao_xyz_camera = np.c_[dx_camera, dy_camera, dz_camera]; 
    velocidade_xyz_camera = np.c_[vx_camera, vy_camera, vz_camera]  

    # Absolute velocities relative to the camera

    velocidades=np.zeros((m,numero_juntas))

    for frame in range(0,m):
    
        if frame == 0:
            anterior = frame;
        else:
            anterior = frame-1;
        
        actual = frame;
    
        for i in range(0,15):
            velocidades[frame,i]= (mat.pdist([[x_camera[actual,i], y_camera[actual,i], z_camera[actual,i]], [x_camera[anterior,i], y_camera[anterior,i], z_camera[anterior,i]]]))/(frame_rate)
    
    velocidades_total_camera = velocidades
    
    return [distancias_total, distancias_ao_torso, velocidades_total, velocidade_xyz, direcao_xyz, angulos, variacao_angulos, variacao_xyz_camera, velocidade_xyz_camera, velocidades_total_camera]

   
def fusao_DBMM(matriz_proba_NB, matriz_proba_SVC):
    peso_NB = 0.6
    peso_SVC = 0.4
    [m,n]=np.shape(matriz_proba_NB)
    resultado = np.zeros((m,n))    
    
    for i in range(0,len(matriz_proba_NB)):
        if i==0:
            resultado[i,:] = matriz_proba_NB[i,:]*0.6+matriz_proba_SVC[i,:]*0.4
        else:
            resultado[i,:] = (resultado[i-1,:] * (matriz_proba_NB[i,:]*0.6+matriz_proba_SVC[i,:]*0.4))
            resultado[i,:] = resultado[i,:] / np.sum(resultado[i,:])

    return resultado        
##################################################### END OF FUNCTIONS #################################################################

rospy.init_node('classifica')

# Load file with the training data
treino = sio.loadmat('treino_torso_camera.mat')
treino = treino['treino']
maxval = treino.max(0)
minval = treino.min(0)

MAIN_FRAME = 'torso_'

listener = tf.TransformListener()

rate = rospy.Rate(30.0)
dados_torso = np.array([])
dados_camera = np.array([])
M = np.array([])
segundos = 0

while not rospy.is_shutdown():
    counter = 0
    frame_list = listener.getFrameStrings()    # tf frame list 

    for frame in frame_list:                   # Count the number of users detected by the openni_tracker 
        if MAIN_FRAME in frame:
            counter+=1    

    
    while not os.path.exists('test_torso.txt'):
        print "Waiting..."
        #time.sleep(3)

    
    if os.path.isfile('test_torso.txt'):
        # read file
        #time.sleep(2)
        segundos+=3
        with open("test_torso.txt") as f:
            num_lines = sum(1 for line in open("test_torso.txt"))
            print num_lines
            while not num_lines>=150:
                time.sleep(0.001)
                num_lines = sum(1 for line in open("test_torso.txt"))
            dados_torso = np.loadtxt(f)
        with open("test_camera.txt") as f_c:
            num_lines = sum(1 for line in open("test_camera.txt"))
            dados_camera = np.loadtxt(f_c)
        print np.shape(dados_torso)  
    else:
        raise ValueError("%s isn't a file!" % file_path)                                            
    
    # Features Extraction 
    start=time.time()
    [distancias_total, distancias_ao_torso, velocidades_total, velocidade_xyz, direcao_xyz, angulos, variacao_angulos, variacao_xyz_camera, velocidade_xyz_camera, velocidades_total_camera] = feature_extraction_torso_camera(dados_torso[0::10,:], dados_camera[0::10,:]);
    
    test = np.c_[distancias_total, velocidades_total, velocidade_xyz_camera, velocidades_total_camera, angulos, variacao_angulos]
    test = test[:, (test != 0).sum(axis=0) > 0]
    
    # Median Filter
    #test = signal.medfilt(test,3)
    
    # Mean Filter
    #test_suavizado = moving_average(test,3) 
    #test_suavizado = np.array(test_suavizado)
    [a,b] = test.shape
    #test= np.resize(test_suavizado,(a,b))
    
    # Normalization
    test = (test-minval)/(maxval-minval)
    
    print np.shape(test)
    
    # NAIVE BAYES
    clfNB = externals.joblib.load('/home/robchair/MarioVieira_ws/src/learning_tf/src/NaiveBayes_torso_camera/NB_clf.pkl')
    predicao_NB = clfNB.predict(test)
    predicao_NB = np.ndarray.tolist(predicao_NB)
    proba_predict_NB = clfNB.predict_proba(test)
    print "Naive Bayes:"
    print proba_predict_NB.mean(0)
    
    # SVC
    clf_svc = externals.joblib.load('/home/robchair/MarioVieira_ws/src/learning_tf/src/SVC_torso_camera/svm_clf.pkl')
    predicao_svc = clf_svc.predict(test)
    predicao_svc = np.ndarray.tolist(predicao_svc)
    proba_predict_svc = clf_svc.predict_proba(test)
    print "SVM:" 
    print proba_predict_svc.mean(0)

    weights = np.array([0.7, 0.3]) 
    proba_predict=0
    for i in range(1,9):
        c = predicao_NB.count(i)
        conta_class[i-1]=c
    
    [proba_final, dbmm_nop] = dbmm(proba_predict_NB, proba_predict_svc, weights, 0) # DBMM
    indexes=np.argmax(proba_final,1)
    
    counts = np.bincount(indexes)
    most_freq = np.argmax(counts)
    prob_correcto = float(np.count_nonzero(indexes==most_freq))/indexes.size
    #proba_final = weights[0]*proba_predict_NB.mean(0)+weights[1]*proba_predict_svc.mean(0)
    #proba_final = fusao_DBMM(proba_predict_NB, proba_predict_svc)
    proba_final = proba_final.mean(0)
    proba_final = np.array([proba_final])
    
    print "Fusion:" 
    print proba_final
    print indexes
    print most_freq
    print prob_correcto
    end=time.time()
    print end-start     # Computation time

    if segundos>0:

        #print np.shape(conta_class)
        #conta_class=np.ndarray.tolist(conta_class)
        proba_final=np.ndarray.tolist(proba_final.mean(axis=0))
        print "Activity: %s" % actividades[proba_final.index(max(proba_final))]
        conta_class = np.zeros((8)) 
        if proba_final.index(max(proba_final)) == 6: # If falling
            os.system("rosrun sound_play say.py 'do you need help?'") 
            #os.system("roslaunch pocketsphinx robocup.launch")
            resposta()
        if proba_final.index(max(proba_final)) == 4: # Se running
            os.system("rosrun sound_play say.py 'it is not allowed to run in this room'")
        if proba_final.index(max(proba_final)) == 5: # Se saltar
            os.system("rosrun sound_play say.py 'it is not allowed to jump in this room'")
        #if proba_final.index(max(proba_final)) == 0: # If walking
            #os.system("rosnode kill /simple_navigation_goals")
            #os.system("rosrun random_navigation_goals follower")
            #process = Popen(["rosrun","random_navigation_goals", "follower"], stdout=PIPE)


        #if a>=27:
        os.remove("test_torso.txt")
        os.remove("test_camera.txt")
        segundos=0
        time.sleep(1)
        #except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         #   continue

        #f.close()
    rate.sleep()
