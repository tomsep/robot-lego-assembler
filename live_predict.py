import cv2 as cv
import tensorflow as tf
import yaml
import time
import numpy as np

from legoassembler.communication import Client
from legoassembler.vision import remote_capture
from legoassembler.utils import load_config


tf.enable_eager_execution()


def load_colors(fname):
    with open(fname, 'r') as f:
        return yaml.safe_load(f.read())



def parse_image(image):
    """ Process png image
    """
    image = tf.convert_to_tensor(image, tf.uint8)
    def _process(_image, reverse_chans=False):


        # Don't use tf.image.decode_image, or the output shape will be undefined
        if reverse_chans:
            _image = tf.reverse(_image, axis=[-1])
        # This will convert to float values in [0, 1]
        _image = tf.image.convert_image_dtype(_image, tf.float32)

        _image = tf.image.resize_images(_image, [160, 160])
        return _image

    image = _process(image, reverse_chans=False)

    with tf.device('/cpu:0'):
        sobel = tf.image.sobel_edges(tf.expand_dims(image, axis=0))
        sobel = tf.squeeze(sobel)
        sobel = tf.reshape(sobel, [160, 160, 6])
        ass_shape = tf.debugging.assert_equal(tf.shape(sobel), tf.constant([160, 160, 6]))
        with tf.control_dependencies([ass_shape]):
            image = tf.concat([image, sobel], axis=2)

    return image


if __name__ == '__main__':
    cfg = load_config()
    rpi_network = cfg['network']['raspi']

    fname = 'color_definitions.yml'
    colors = load_colors(fname)
    client = Client()

    client.connect(rpi_network['ip'], rpi_network['port'])

    cam_params = cfg['camera_parameters']

    model_path = './tfmodels/sobel-unet-color-psize/model.h5'

    model = tf.keras.models.load_model(model_path, compile=False)


    while True:
        img = remote_capture(client, cam_params)
        #img = cv.imread('./imageset/validation/1_img.png', cv.IMREAD_COLOR)
        cv.imshow('orig', img)


        shape = np.shape(img)
        h_cent, w_cent = (shape[0] // 2, shape[1] // 2)
        roi = img[h_cent-200:h_cent+200, w_cent-200:w_cent+200, :]
        img_in = parse_image(roi)
        roi_rszized = cv.resize(roi, (160, 160))
        cv.imshow('ROI', roi)
        cv.imshow('ROI 160x', roi_rszized)

        start = time.time()

        # bgr to rgb
        #roi_rszized = np.flip(roi_rszized, axis=2)
        roi_rszized = roi_rszized.astype('float') / 255

        pred = model.predict(np.expand_dims(img_in, axis=0), batch_size=1)
        print('Prediction elapsed time: {}'.format(time.time() - start))

        pred = cv.resize(pred[0], (int(400), int(400)))
        cv.imshow('prediction', pred)

        cv.waitKey(30)
        time.sleep(0.2)


