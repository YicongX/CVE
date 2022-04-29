# import qrcode
import cv2
import numpy as np
from pyzbar.pyzbar import decode


# def generateQRCode(x, filename):
#     # input data
#     input_data = x
#     # generate qr code
#     qr = qrcode.make(input_data)
#     # save img to a file
#     qr.save(filename)


def detectQR(img):
#     # initialize the cv2 QRCode detector
#     detector = cv2.QRCodeDetector()
#     # detect and decode
#     data, bbox, straight_qrcode = detector.detectAndDecode(img)
#     # if there is a QR code
#     if bbox is not None:
#         # display the image with lines
#         print(f"QRCode data:\n{data}")
#         pts = bbox.reshape(-1, 1, 2).astype(np.int)
#         print(pts)
#         cv2.polylines(img, [pts], True, color=(255, 0, 0), thickness=2)
    #initialize the cv2 QRCode detector
    for qr in decode(img):
        data = qr.data.decode('utf-8')
        pts = np.array([qr.polygon], np.float32)
        print(pts)
        pts.reshape(-1,1,2)
        cv2.polylines(img, [pts], True, color=(255, 0, 0), thickness=2)

    return img


def videoQRRead(vd_input=None):
    # initialize the cam
    # Synchronous reading
    cap = cv2.VideoCapture(1)
    # video input
    # cap = cv2.VideoCapture(vd_input)
    while True:
        _, img = cap.read()
        # detect and decode
        img_out = detectQR(img)
        cv2.imshow("img_out", img_out)
        # press q or esc to stop
        if cv2.waitKey(20) == ord("q") or cv2.waitKey(20) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


def main():
    # for i in range(6):
    #     generateQRCode(i, str(i) + "_QR.png")
    # vd_input = 'QR(2).mp4'
    # cam = 1 # 1 for external webcam, 0 for internal cam
    # vd_input = cv2.VideoCapture(cam)
    # if not vd_input: print("!!!Failed VideoCapture: invalid camera source!!!")
    videoQRRead()


if __name__ == "__main__":
    main()
