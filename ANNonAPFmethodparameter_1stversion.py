# -*- coding: utf-8 -*-
"""
Created on Sat Aug  7 00:22:23 2021

@author: Phoenix WANG
contanct:shadowdouble76@gmail.com
"""
import matplotlib.pyplot as plt
import tensorflow as tf
import pandas as pd
from tensorflow import keras
from tensorflow.keras import layers


dataset_path1 = '..........' #file_path of saving trainning data
column_names1 = ['ETA','rr','path_length']
raw_dataset1 = pd.read_csv(dataset_path1, names=column_names1,na_values = "?", comment='\t' ,
                           sep = ",", skipinitialspace=True)



datasets = raw_dataset1.copy()
datasets = datasets.dropna()
train_datas = datasets.sample(frac=0.8,random_state=0)
test_datas = datasets.drop(train_datas.index)


train_stats = train_datas.describe()
train_stats.pop("path_length")
train_stats = train_stats.transpose()
train_labels = train_datas.pop('path_length')
test_labels = test_datas.pop('path_length')

def norm(x):
    return (x-train_stats['mean']) / train_stats['std']
normed_train_datas = norm(train_datas)
normed_test_datas = norm(test_datas)

def build_model():
    model = keras.Sequential([
        layers.Dense(64,activation='relu',input_shape=[len(train_datas.keys())]),
        layers.Dense(64,activation='relu'),
        layers.Dense(1)
        ])
    optimizer = tf.keras.optimizers.RMSprop(0.001)
    
    model.compile(loss='mse',
                  optimizer = optimizer,
                  metrics=['mae','mse'])
    return model

class PrintDot(keras.callbacks.Callback):
    def on_epoch_end(self,epoch,logs):
        if epoch%100 == 0:
            print('')
        print('.',end='')
        
def plot_history(history):
    hist = pd.DataFrame(history.history)
    hist['epoch'] = history.epoch
    
    plt.figure
    plt.xlabel('Epoch')
    plt.ylabel('Mean Abs Error [path_length]')
    plt.plot(hist['epoch'],hist['mae'],label='Train Error')
    plt.plot(hist['epoch'],hist['val_mae'],label='Val Error')
    plt.ylim([0,5])
    plt.legend()
    
    plt.figure()
    plt.xlabel('Epoch')
    plt.ylabel('Mean Square Error [$path_length^2$]')
    plt.plot(hist['epoch'],hist['mse'],label='Train Error')
    plt.plot(hist['epoch'],hist['val_mse'],label = 'Val Error')
    
EPOCHS = 1000
model = build_model()
early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
history = model.fit(normed_train_datas,train_labels,epochs=EPOCHS,
                     validation_split = 0.2, verbose=0, callbacks=[early_stop,PrintDot()])

plot_history(history)

loss, mae, mse = model.evaluate(normed_test_datas,test_labels,verbose=2)

print("Testing set Mean Abs Error:{:5.2f} path_length".format(mae))

test_predictions = model.predict(normed_test_datas).flatten()

plt.scatter(test_labels, test_predictions)
plt.xlabel('True Values [path_length]')
plt.ylabel('Predictions [path_length]')
plt.axis('equal')
plt.xlim([0,plt.xlim()[1]])
plt.ylim([0,plt.ylim()[1]])
_ = plt.plot([-100, 100], [-100, 100])
