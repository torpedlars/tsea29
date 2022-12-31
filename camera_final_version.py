from picamera import PiCamera
from time import sleep
from PIL import Image
import zbarlight
camera = PiCamera()

def take_picture():
        #start camera
        camera.start_preview()
        #camera warmup
        sleep(0.3)
        #capture image
        camera.capture('/home/g14/Desktop/Project/barcode.png')
        #very important to close camera
        camera.stop_preview()
        file_path = '/home/g14/Desktop/Project/barcode.png'
        with open(file_path, 'rb') as image_file:
            image = Image.open(image_file)
            image.load()
        new_image = zbarlight.copy_image_on_background(image, color=zbarlight.WHITE)
        #new_image = new_image.rotate(180)
        sleep(0.2)
        return new_image
    
def find_pallet(pallet):
#find_pallet():
    NoPicture = True
    #pallet = b'12'
    print("in find_pallet function")

    while NoPicture:
        new_image = take_picture()
        sleep(0.2)
        new_image2 = take_picture()
        codes = zbarlight.scan_codes(['code128'], new_image)
        codes2 = zbarlight.scan_codes(['code128'], new_image2)
        print('Barcodes: %s' % codes)
        print('Barcodes: %s' % codes2)
        if codes != None and codes == codes2:
            pos = 3
            for code in codes:
                print(code)
                if int(code) == pallet:
                    NoPicture = False
                    break
                else:
                    pos -= 1
    return pos #returns the position of the pallet
