#  Created by Luis Alejandro (alejand@umich.edu)

import numpy as np

from sklearn.metrics import accuracy_score
from sklearn.metrics import f1_score
from sklearn.metrics import recall_score
from sklearn.metrics import precision_score

# Utility function to report best scores in the cross-validation
def report_grid_search(results, n_top=3):
    """ Report the results from a grid search using GridSearchCV implementation

        Parameters: 
        results: The resulting object from GridSearchCV fit method
        n_top (int): Top ranks to report
    """
    for i in range(1, n_top + 1):
        candidates = np.flatnonzero(results['rank_test_score'] == i)
        for candidate in candidates:
            print("Model with rank: {0}".format(i))
            print("Mean validation score: {0:.3f} (std: {1:.3f})".format(
                  results['mean_test_score'][candidate],
                  results['std_test_score'][candidate]))
            print("Parameters: {0}".format(results['params'][candidate]))
            print("")
            
# Utility function to report the test peformance
def report_classification(y_pred, y_true, avg='binary',title='Test'):
    """ Report the classification results using accuracy, f1 score, recall and precision

        Parameters: 
        y_pred: Vector of predicted outputs
        y_true: Vector of true outputs
        avg: Indicates what average mode (binary, micro, macro) to use
        title: Title shown in the output
    """
    
    print(title, '(Metrics): ')
    print('')
    print('Accuracy: ', '%.2f' % accuracy_score(y_pred,y_true))
    print('F1 Score: ', '%.2f' % f1_score(y_pred,y_true, average=avg))
    print('Recall: ', '%.2f' % recall_score(y_pred,y_true, average=avg))
    print('Precision: ', '%.2f' % precision_score(y_pred,y_true, average=avg))
    
def report_feature_ranking(rank, feature_names, print_count = 6):
    '''
        Prints out the feature with its corresponding rank value
        
        Arguments:
            rank(nparray): a rank of each feature
            feature_names(list): a list of all feature names
            print_count(int): how many features to print. It picks half from the top, half form the bottom to print
    '''
    
    indexes = rank.flatten().argsort()
    d = len(indexes)
    if print_count > d:
        print_count = d
    
    # prints top features
    top = int(np.ceil(print_count / 2))
    for i in range(1, top + 1):
        print('Feature ranked %d is (%s) with value %lf' % (i,feature_names[indexes[-i]],rank[indexes[-i]]))
    
    # prints the points if needed
    if d > print_count:
        print('.\n.\n.\n')
        
    # prints bottom features
    bottom = print_count - top
    for i in range(bottom-1,-1,-1):
        print('Feature ranked %d is (%s) with value %lf' % (d - i,feature_names[indexes[i]],rank[indexes[i]]))
    
    
    
    
    
    