# OpenCV program to perform Edge detection in real time
# import libraries of python OpenCV
# where its functionality resides
import cv2

# np is an alias pointing to numpy library
import numpy as np

# capture frames from a camera
cap = cv2.VideoCapture(0)

def rotate(image, angle):
    # grab the dimensions of the image and then determine the
    # center
    (h, w) = image.shape[:2]
    (cX, cY) = (w / 2, h / 2)

    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., the rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)

    # perform the actual rotation and return the image
    return cv2.warpAffine(image, M, (w, h))

def avg_lines(frame, lines):
    import numpy.polynomial.polynomial as poly
    lane_line = []
    ht, wt, _ = frame.shape

    ll_s = []
    lr_s = []
    rr_s = []
    rl_s = []

    if lines is None:
        print('No lines to detect')
        return lane_line

    for line in lines:
        for x1,y1,x2,y2 in line:
            #find slope using points and categorize left from right
            fit = poly.polyfit((x1,x2), (y1,y2), 1)
            slope = fit[1]
            intercept = fit[0]
            if slope < 0:
                if x2 < int(wt*(2/3)):
                    ll_s.append((slope,intercept))                              #neg slope on left side (straight/right)
                else:
                    lr_s.append((slope,intercept))                              #neg slope on right side (turn right)
            elif slope > 0:
                if x2 >  int(wt*(1/3)):
                    rr_s.append((slope,intercept))                              #pos slope on right side (straight/left)
                else:
                    rl_s.append((slope,intercept))                              #pos slope on left side (turn left)
            else:
                pass


    # average lines (slope, int)
    avglr = np.mean(lr_s, axis = 0)
    print("\nRight", avglr)
    avgll = np.mean(ll_s, axis = 0)
    print("Strt/Right", avgll)                         #debug
    avgrr = np.mean(rr_s, axis = 0)
    print("Strt/Left", avgrr)                         #debug
    avgrl = np.mean(rl_s, axis = 0)
    print("Left", avgrl)

    #create lane lines based on categorization
    if len(ll_s) > 0:
        slopel = avgll[0]
        intl = avgll[1]
        y1 = ht
        y2 = int(y1/2)
        x1 = max(-wt,min(2*wt,(y1-intl)/slopel))
        x2 = max(-wt,min(2*wt,(y2-intl)/slopel))
        left_lane = [int(x1),y1,int(x2),y2]
        lane_line.append([left_lane])
    elif len(rl_s) > 0:
        slopel = avgrl[0]
        intl = avgrl[1]
        y1 = ht
        y2 = int(y1/2)
        x1 = max(-wt,min(2*wt,(y1-intl)/slopel))
        x2 = max(-wt,min(2*wt,(y2-intl)/slopel))
        left_lane = [int(x1),y1,int(x2),y2]
        lane_line.append([left_lane])

    if len(rr_s) > 0:
        sloper = avgrr[0]
        intr = avgrr[1]
        y1 = ht
        y2 = int(y1/2)
        x1 = max(-wt,min(2*wt,(y1-intr)/sloper))
        x2 = max(-wt,min(2*wt,(y2-intr)/sloper))
        right_lane = [int(x1),y1,int(x2),y2]
        lane_line.append([right_lane])
    elif len(lr_s) > 0:
        sloper = avglr[0]
        intr = avglr[1]
        y1 = ht
        y2 = int(y1/2)
        x1 = max(-wt,min(2*wt,(y1-intr)/sloper))
        x2 = max(-wt,min(2*wt,(y2-intr)/sloper))
        right_lane = [int(x1),y1,int(x2),y2]
        lane_line.append([right_lane])

    #print("lane lines", lane_line)              #debug
    return lane_line

# loop runs if capturing has been initialized
while(1):

	# reads frames from a camera
    ret, frame = cap.read()
    #cv2.imshow('Original', frame)

    # rotate frame
    frame1 = rotate(frame, 180)
    #cv2.imshow('Rotated',frame1)

	# converting BGR to HSV
    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)

    #color detection
    low_red = np.array([95,30,90]) #blue
    up_red = np.array([145,255,255])
    mask = cv2.inRange(hsv, low_red, up_red)
    res = cv2.bitwise_and(hsv, hsv, mask = mask)
    #cv2.imshow('Color', res)

    #crop the top of the image away
    crop = np.zeros(res.shape, dtype = 'uint8')
    cv2.rectangle(crop, (0, 240), (640, 480), (255, 255, 255), -1)
    crop_im = cv2.bitwise_and(src1 = res, src2 = crop)

    # finds edges in the input image image and
	# marks them in the output map edges
    crop_g = cv2.cvtColor(crop_im, cv2.COLOR_HSV2BGR)
    crop_g = cv2.cvtColor(crop_g, cv2.COLOR_BGR2GRAY) #converts from HSV to grayscale, attempt
    edges = cv2.Canny(crop_g ,90,180)
    #cv2.imshow('Edges', edges)

    # line detection using Hough transform
    minLineLength = 50
    maxLineGap = 25
    lines = cv2.HoughLinesP(edges,1,np.pi/180,40,minLineLength,maxLineGap)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(frame1,(x1,y1),(x2,y2),(0,255,0),2)
    else:
        pass

    cv2.imshow('Hough', frame1)

    #find lane lines from detected lines
    lanes = avg_lines(frame1, lines)
    #print("lanes", lanes)                  #debug
    # draw calculated lanes
    if lanes is not None:
        for line in lanes:
            for x1,y1,x2,y2 in line:
                cv2.line(frame1, (x1,y1), (x2,y2), (0,255,0), 10)

    cv2.line(frame1, (int(frame.shape[1]/2),int(frame.shape[0])),(int(frame.shape[1]/2),0),(255,0,0),2)
    cv2.imshow("lanes", frame1)

	# Wait for 'q' key to stop
    k = cv2.waitKey(2) & 0xFF
    if k == 113:
        break
    else:
        pass

# Close the window
cap.release()

# De-allocate any associated memory usage
cv2.destroyAllWindows()

#color = np.uint8([[[b,g,r]]])
#colorHSV = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
#range [h-10, 100, 100] to [h+10, 255, 255]
