from picamera import PiCamera
from time import sleep
from PIL import Image
import zbarlight
camera = PiCamera()

def take_picture():
        #start camera
        camera.start_preview()
        #camera warmup
        sleep(2)
        #capture image
        camera.capture('/home/torpedlars/Desktop/barcode.png')
        #very important to close camera
        camera.stop_preview()
        file_path = '/home/torpedlars/Desktop/barcode.png'
        with open(file_path, 'rb') as image_file:
            image = Image.open(image_file)
            image.load()
        new_image = zbarlight.copy_image_on_background(image, color=zbarlight.WHITE)
        sleep(2)
        return new_image

def find_pallet(pallet):
    NoPicture = True
    #pallet = b'10'
    
    while NoPicture:
        new_image = take_picture()
        sleep(2)
        new_image2 = take_picture()
        codes = zbarlight.scan_codes(['code128'], new_image)
        codes2 = zbarlight.scan_codes(['code128'], new_image2)
        #print('Barcodes: %s' % codes)
        if codes != None and codes == codes2:
            pos = 0
            for code in codes:
                if codes == pallet:
                    NoPicture = False
                    break
                else:
                    pos += 1
    return pos #returns the position of the pallet