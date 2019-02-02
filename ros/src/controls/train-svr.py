import numpy as np
import sys
import sklearn.svm
import pickle

read = np.genfromtxt(sys.argv[1], delimiter=',')
read = read[2:,1:]

X= read[:,1:]
Y= read[:,0]

svr = sklearn.svm.SVR()
svr.fit(X,Y)
pickle.dump(svr, open('svr-model','wb'), protocol=2)
print(svr.predict([[-1.18580314112,-3.07527014201,-3.32922857805]]))