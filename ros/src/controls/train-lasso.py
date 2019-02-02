# Import function to create training and test set splits
from sklearn.cross_validation import train_test_split

# Import function to automatically create polynomial features! 
from sklearn.preprocessing import PolynomialFeatures

# Import Linear Regression and a regularized regression function
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import LassoCV
import numpy as np
# Finally, import function to make a machine learning pipeline
from sklearn.pipeline import make_pipeline
# Alpha (regularization strength) of LASSO regression
lasso_eps = 0.001
lasso_nalpha=100
lasso_iter=1000000
import sys
# Min and max degree of polynomials features to consider
degree_min = 4
degree_max = 15

test_set_fraction=0.1
# Test/train split
read = np.genfromtxt(sys.argv[1], delimiter=',')
read = read[2:,:]
X_train, X_test, y_train, y_test = train_test_split(read[:,1:], read[:,0],test_size=test_set_fraction)

# Make a pipeline model with polynomial transformation and LASSO regression with cross-validation, run it for increasing degree of polynomial (complexity of the model)
best_score = 0
import pickle
for degree in range(degree_min,degree_max+1):
    model = make_pipeline(PolynomialFeatures(degree, interaction_only=False), LassoCV(eps=lasso_eps,n_alphas=lasso_nalpha,max_iter=lasso_iter,
normalize=True,cv=5))
    model.fit(X_train,y_train)
    test_pred = np.array(model.predict(X_test))
    RMSE=np.sqrt(np.sum(np.square(test_pred-y_test)))
    test_score = model.score(X_test,y_test)
    print(RMSE)
    print(test_score)
    if test_score > best_score:
      print("Writing model")
      pickle.dump(model, open('best_model','wb'), protocol=2)
      best_score = test_score
    print(model.predict([[0, 8,-5], [1,8, 0], [0,0, -5], [0,0,5]]))
