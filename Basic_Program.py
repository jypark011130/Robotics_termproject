# import the module for opencv
import cv2

#connect the camera
cam_h = cv2.VideoCapture(1)

# to check if the camera is connected correctly
if cam_h.isOpened():
    while True:
        # to read the images from camera
        ret, img = cam_h.read()
        if ret :
            #to visualize the images
            cv2.imshow("webcam", img)
            if cv2.waitKey(1) != -1:
                break

        else:
            print("Frame Error")
            break
else:
    print("Cannot open camera!")


#to disconnect the camera
cam_h.release()
#to destroy the windows
cv2.destroyAllWindows()

#Change the Format##
#to check if the camera is connected correctly
if cam_h.isOpened():
    while True:
        #to read the images from the camera
        ret, img = cam_h.read()
        if ret:
            #to do something
            dst = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            dst2 = cv2.cvtColor(img, cv2.COLOR_RBG2HSV)
            r, g, b = cv2.split(img)
            h, s, v = cv2.split(dst2)
            #to visualize the images
            cv2.imshow("webcam", img)
            cv2.imshow("modified", dst)
            cv2.imshow("red", r)
            cv2.imshow("hue", h)
            if cv2.waitKey(1) != -1:
                break


##Edge Detection##
#to check if the camera is connected correctly
if cam_h.isOpened():
    while True:
        #to read the images from camera
        ret, img = cam_h.read()
        if ret:
            #to do something
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            ret, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

            counters,hierachy = cv2.findCounters(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            src = img

            for i in range(len(contours)):
                cv2.drawContours(src, [contours[i]], 0, (0, 0, 255), 2)
                #cv2.putText(src, stri(i), tuple(contours[i][0][0]), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 1)
                #print(i, hierachy[0][i])

            #to visualize the images
            cv2.imshow("webcam", img)
            cv2.imshow("gray", gray)
            cv2.imshow("thresholding", binary)
            cv2.imshow("webcam+contours", src)

            if cv2.waitKey(1) != -1:
                break



##Contour Moment##
#to check if the camera is connected correctly
if cam_h.isOpened():
    while True:
        #to read the images from camera
        ret, img = cam_h.read()
        if ret:
            #to do something
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            ret, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

            counters,hierachy = cv2.findCounters(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            src = img
            for i in contours:
                M = cv2.moments(i)

                if M['m00'] >50:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])

                    cv2.circle(src, (cX, cY), 3, (255, 0, 0), -1)

                cv2.drawContours(src, [i], 0, (0, 0, 255), 2)


            #to visualize the images
            cv2.imshow("webcam", img)
            cv2.imshow("gray", gray)
            cv2.imshow("thresholding", binary)
            cv2.imshow("webcam+contours", src)