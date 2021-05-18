import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import random
import cv2
from sklearn.preprocessing import OneHotEncoder
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.layers import Conv2D, MaxPooling2D, BatchNormalization
from tensorflow.keras.layers import Activation, Flatten, Dropout, Dense
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from sklearn.metrics import classification_report


def load_data(path, filename):
    # turns each row into a list, put them in a whole list and shuffles data
    file_path = os.path.join(path, filename)
    csvfile = pd.read_csv(file_path, delimiter=',')
    rows = [list(row) for row in csvfile.values]
    random.shuffle(rows)

    # preprocess the data -> getting the class ID and image path
    labels = []
    images = []
    for (i, row) in enumerate(rows):
        label = row[-2]
        labels.append(int(label))

        image_path = os.path.sep.join([path, row[-1]])
        image = cv2.imread(image_path)
        image = cv2.resize(image, (32, 32))
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        images.append(final)

    labels = np.array(labels)
    images = np.array(images)

    return images, labels


train_X, train_Y = load_data("./", "Train.csv")
test_X, test_Y = load_data("./", "Test.csv")

# scaling images to values [0 ... 1]
train_X = train_X.astype("float32") / 255.0
test_X = test_X.astype("float32") / 255.0

labels_num = len(np.unique(train_Y))

# normalize labels
class_totals = train_Y.sum(axis=0)
class_weight = class_totals.max() / class_totals


EPOCHS_NUM = 30
INIT_LEARNING_RATE = 1e-3
BATCH_SIZE = 64


model = Sequential()
channel_dim = -1

model.add(Conv2D(8, (5, 5), padding="same", activation='relu', input_shape=(32, 32, 3)))
model.add(BatchNormalization(axis=channel_dim))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(16, (3, 3), padding="same", activation='relu'))
model.add(BatchNormalization(axis=channel_dim))
model.add(Conv2D(16, (3, 3), padding="same", activation='relu'))
model.add(BatchNormalization(axis=channel_dim))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(32, (3, 3), padding="same", activation='relu'))
model.add(BatchNormalization(axis=channel_dim))
model.add(Conv2D(32, (3, 3), padding="same", activation='relu'))
model.add(BatchNormalization(axis=channel_dim))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(BatchNormalization())
model.add(Dropout(0.5))

model.add(Dense(labels_num, activation='softmax'))

print("Compiling Model")
optim=Adam(lr=INIT_LEARNING_RATE, decay=INIT_LEARNING_RATE/(EPOCHS_NUM*0.5))

model.compile(loss="sparse_categorical_crossentropy", optimizer=optim, metrics=["accuracy"])


aug=ImageDataGenerator(
    rotation_range=10,
    zoom_range=0.15,
    width_shift_range=0.1,
    height_shift_range=0.1,
    shear_range=0.15,
    horizontal_flip=False,
    vertical_flip=False,
    fill_mode="nearest"
)

print("Training Network")
H=model.fit_generator(
    aug.flow(train_X, train_Y, batch_size=BATCH_SIZE),
    validation_data=(test_X, test_Y),
    steps_per_epoch=train_X.shape[0]//BATCH_SIZE,
    epochs=EPOCHS_NUM,
    class_weight=class_weight,
    verbose=1
)

# serialize model to JSON
model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)
# serialize weights to HDF5
model.save_weights("model1.h5")
print("Saved model to disk")

csvfile_names = pd.read_csv("signnames.csv", delimiter=',')
sign_names2 = [list(row) for row in csvfile_names.values]
sign_names2 = np.array(sign_names2)
sign_names2 = sign_names2[:, 1]

path_to_model = "./model.h5"
predictions = model.predict(test_X, batch_size=BATCH_SIZE)
model.save(path_to_model)
# print(classification_report(test_Y.argmax(axis=1), predictions.argmax(axis=1), target_names=sign_names2))
