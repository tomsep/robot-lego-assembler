from __future__ import division, print_function
import tensorflow as tf
from tensorflow.keras import layers
import numpy as np
import cv2 as cv
import time


def parse_function(image, label):
    """ Process png images 'image' and 'label'
    Parameters should be filepaths.


    Returns
    -------
    Tensor, Tensor
        Color images RGB

    """

    def _process(_fname, reverse_chans=False):

        _image_string = tf.read_file(_fname)

        # Don't use tf.image.decode_image, or the output shape will be undefined
        _image = tf.image.decode_png(_image_string, channels=3)
        if reverse_chans:
            _image = tf.reverse(_image, axis=[-1])
        # This will convert to float values in [0, 1]
        _image = tf.image.convert_image_dtype(_image, tf.float32)

        _image = tf.image.resize_images(_image, [160, 160])
        return _image

    image = _process(image, reverse_chans=True)
    label = _process(label, reverse_chans=True)

    with tf.device('/cpu:0'):
        sobel = tf.image.sobel_edges(tf.expand_dims(image, axis=0))
        sobel = tf.squeeze(sobel)
        sobel = tf.reshape(sobel, [160, 160, 6])
        ass_shape = tf.debugging.assert_equal(tf.shape(sobel), tf.constant([160, 160, 6]))
        with tf.control_dependencies([ass_shape]):
            image = tf.concat([image, sobel], axis=2)

    return image, label


def train_preprocess(image, label):
    """ Data augmentation

    Random flipping, brightness and saturation.

    Parameters
    ----------
    image : Tensor
    label : Tensor

    Returns
    -------
    Tensor, Tensor

    """

    image = image[:, :, :3]

    # Randomize left-right flip
    do_flip = tf.random_uniform([]) > 0.5
    image = tf.cond(do_flip, lambda: tf.image.flip_left_right(image), lambda: image)
    label = tf.cond(do_flip, lambda: tf.image.flip_left_right(label), lambda: label)

    # Randomize up-down flip
    do_flip = tf.random_uniform([]) > 0.5
    image = tf.cond(do_flip, lambda: tf.image.flip_up_down(image), lambda: image)
    label = tf.cond(do_flip, lambda: tf.image.flip_up_down(label), lambda: label)

    # Randomize brightness and saturation
    image = tf.image.random_brightness(image, max_delta=32.0*2 / 255.0)
    image = tf.image.random_saturation(image, lower=0.25, upper=2)

    # Make sure the image is still in [0, 1]
    image = tf.clip_by_value(image, 0.0, 1.0)

    with tf.device('/cpu:0'):
        sobel = tf.image.sobel_edges(tf.expand_dims(image, axis=0))
        sobel = tf.squeeze(sobel)
        sobel = tf.reshape(sobel, [160, 160, 6])
        ass_shape = tf.debugging.assert_equal(tf.shape(sobel), tf.constant([160, 160, 6]))
        with tf.control_dependencies([ass_shape]):
            image = tf.concat([image, sobel], axis=2)

    ass_shape = tf.debugging.assert_equal(tf.shape(image), tf.constant([160, 160, 9], tf.int32))

    with tf.control_dependencies([ass_shape]):
        image = tf.identity(image)

    return image, label


def convdown(inputs, depth, dropout=0):
    kernel = (3, 3)
    x = layers.MaxPool2D((2, 2), padding='valid')(inputs)
    x = layers.Conv2D(depth, kernel, padding='same', activation='selu')(x)
    x = layers.Dropout(dropout)(x)
    x = layers.Conv2D(depth, kernel, padding='same', activation='selu')(x)
    x = layers.Dropout(dropout)(x)
    return x


def convup(left_in, down_in, depths, end_softmax=False, dropout=0):
    kernel = (3, 3)

    down_in = layers.UpSampling2D((2, 2))(down_in)
    x = layers.Concatenate()([left_in, down_in])
    x = layers.Conv2D(depths[0], kernel, padding='same', activation='selu')(x)
    x = layers.Dropout(dropout)(x)
    if end_softmax:
        act = 'softmax'
    else:
        act = 'selu'
    x = layers.Conv2D(depths[1], kernel, padding='same', activation=act)(x)
    x = layers.Dropout(dropout)(x)
    return x


def unet_full(depth_factor=1, dropout=0, lrate=0.001):
    kernel = (3, 3)

    depth_factor = int(depth_factor)
    inputs = tf.keras.Input(shape=(160, 160, 9))

    level_0 = layers.Conv2D(32, kernel, padding='same', activation='selu')(inputs)
    level_1 = convdown(level_0, 64 // depth_factor, dropout=dropout)
    level_2 = convdown(level_1, 128 // depth_factor, dropout=dropout)
    level_3 = convdown(level_2, 256 // depth_factor, dropout=dropout)
    level_4 = convdown(level_3, 256 // depth_factor, dropout=dropout)

    level_4_end = layers.Conv2D(512 // depth_factor, kernel, padding='same', activation='selu')(level_4)
    level_4_end = layers.Dropout(dropout)(level_4_end)
    level_4_end = layers.Conv2D(256 // depth_factor, kernel, padding='same', activation='selu')(level_4_end)
    level_4_end = layers.Dropout(dropout)(level_4_end)

    level_3_end = convup(left_in=level_3, down_in=level_4_end, depths=(256 // depth_factor, 128 // depth_factor),
                         dropout=dropout)
    level_2_end = convup(left_in=level_2, down_in=level_3_end, depths=(128 // depth_factor, 64 // depth_factor),
                         dropout=dropout)
    level_1_end = convup(left_in=level_1, down_in=level_2_end, depths=(64 // depth_factor, 32 // depth_factor),
                         dropout=dropout)

    level_0_end = convup(left_in=level_0, down_in=level_1_end, depths=(32 // depth_factor, 3),
                         dropout=dropout)
    output = layers.Activation('softmax')(level_0_end)

    model = tf.keras.Model(inputs=inputs, outputs=output)

    model.compile(optimizer=tf.keras.optimizers.Adam(lrate),
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])
    return model


if __name__ == '__main__':
    """ Train a neural network. Allows continuing or starting a new random initialization.
    """


    batch_size = 13
    nm_tr_images = 40
    nm_valid_images = 6
    lrate = 0.0005
    folder = '../imageset/training/'
    valid_folder = '../imageset/validation/'
    save = True
    load = False
    train = True


    save_dir = '../tfmodels/'
    model_name = 'sobel-unet-color-psize'
    model_dir = save_dir + model_name + '/'

    train_fnames = (np.array([folder + '{}_img.png'.format(x) for x in range(1, nm_tr_images + 1)] * 2),
                    np.array([folder + '{}_label.png'.format(x) for x in range(1, nm_tr_images + 1)] * 2))
    dataset = tf.data.Dataset.from_tensor_slices(train_fnames)
    dataset = dataset.shuffle(len(train_fnames[0]))
    dataset = dataset.map(parse_function, num_parallel_calls=4)
    dataset = dataset.map(train_preprocess, num_parallel_calls=4)
    dataset = dataset.batch(batch_size)
    dataset = dataset.prefetch(4).repeat()
    iter = dataset.make_one_shot_iterator().get_next()


    # Validation dataset
    valid_fnames = (np.array([valid_folder + '{}_img.png'.format(x) for x in range(1, nm_valid_images + 1)]),
                    np.array([valid_folder + '{}_label.png'.format(x) for x in range(1, nm_valid_images + 1)]))
    test_dataset = tf.data.Dataset.from_tensor_slices(valid_fnames)
    test_dataset = test_dataset.map(parse_function, num_parallel_calls=4)
    test_dataset = test_dataset.batch(1)
    test_dataset = test_dataset.prefetch(3).repeat()
    test_iter = test_dataset.make_one_shot_iterator().get_next()


    if load:
        model = tf.keras.models.load_model(model_dir + 'model.h5', compile=False)
        model.compile(optimizer=tf.keras.optimizers.Adam(lrate),
                              loss='categorical_crossentropy',
                              metrics=['accuracy'])
        print('Loaded model from "{}"'.format(model_dir))
    else:
        model = unet_full(depth_factor=2, dropout=0.2, lrate=lrate)
        #model = unet()

    def _display(_img, name): # display image using opencv
        _img = cv.resize(_img, (int(300), int(300)))
        cv.imshow(name, _img)


    if train:
        if input('Start training (save:{}, load:{}) [Y/n]?: '.format(save, load)) == 'Y':
            tboard_call = tf.keras.callbacks.TensorBoard(log_dir=model_dir + '/logs/', batch_size=batch_size)
            if save:
                period = 1
            else:
                period = 0
            save_call = tf.keras.callbacks.ModelCheckpoint(model_dir + 'model.h5', verbose=1, monitor='loss',
                                                           period=period, save_best_only=True)
            early_stop = tf.keras.callbacks.EarlyStopping(monitor='loss', patience=20, verbose=1)
            while True:

                model.fit(dataset, steps_per_epoch=100, epochs=9999, callbacks=[tboard_call, save_call, early_stop],
                          validation_data=test_dataset, validation_steps=nm_valid_images, verbose=1)

                if save:
                    model = tf.keras.models.load_model(model_dir + 'model.h5', compile=False)
                    model.compile(optimizer=tf.keras.optimizers.Adam(lrate),
                                          loss='categorical_crossentropy',
                                          metrics=['accuracy'])
                    print('Re-loaded model from "{}"'.format(model_dir))
        else:
            exit()
    else:


        sess = tf.Session()

        for i in range(nm_valid_images):
            orig = sess.run(test_iter)
            img = orig[0][0][:, :, :3]
            sobel = orig[0][0][:, :, 3:]
            label = orig[1][0]

            _display(img, 'img')
            _display(label, 'label')

            _display(sobel[:, :, :1], 'sobel_a')
            _display(sobel[:, :, 1:2], 'sobel_b')
            # _display(sobel[:, :, 2:3], 'sobel_c')
            # _display(sobel[:, :, 3:4], 'sobel_d')
            # _display(sobel[:, :, 4:5], 'sobel_e')
            # _display(sobel[:, :, 5:], 'sobel_f')

            start = time.time()
            pred = model.predict(np.array([orig[0][0]]), batch_size=1)
            print('Prediction elapsed time: {}'.format(time.time() - start))
            _display(pred[0], 'pred')


            cv.waitKey(75)
            input('Press anything to continue: ')
        sess.close()






