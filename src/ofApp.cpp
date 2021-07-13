#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {

    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofSetWindowPosition(ofGetScreenWidth() / 18., ofGetScreenHeight() / 18.);
    ofSetWindowShape(ofGetScreenWidth() / 18. * 16, ofGetScreenHeight() / 18. * 16);
    ofBackground(0);
    ofSetCircleResolution(32);
    ofSetSphereResolution(32);
    glPointSize(2);
    
    //setup frame
    xml.load("/Users/tskokmt/Documents/PROJECT/FRAME FREE/DATA/frameData.xml");
    frameL = ofRectangle(0, 0, xml.getValue("frameLW", 1340), xml.getValue("frameLH", 460));
    frameC = ofRectangle(frameL.width, 0, xml.getValue("frameCW", 1140), xml.getValue("frameCH", 460));
    frameR = ofRectangle(frameL.width + frameC.width, 0, xml.getValue("frameRW", 1340), xml.getValue("frameRH", 460));
    frame = ofRectangle(0, 0, frameC.width + frameL.width + frameR.width, frameC.height);
    
    //load kinectIndex
    xml.clear();
    xml.load("kinectIndexSaveData.xml");
    kinectIndex = xml.getValue("kinectIndex", 0);
    
    //setup kinect
    kinect.init();
    mode = kinect.open(kinectIndex);
    kinect.setRegistration(true);
    kinectWidth = kinect.width, kinectHeight = kinect.height;
    
    //setup camera
    easyCam.setFov(60);
    easyCam.setNearClip(.1);
    easyCam.setFarClip(100000);
    
    //setup pointCloud
    pointCloud.setMode(OF_PRIMITIVE_POINTS);
    pointCloudZ.setMode(OF_PRIMITIVE_POINTS);
    pointCloudX.setMode(OF_PRIMITIVE_POINTS);
    
    //silhouette
    silhouetteFbo.allocate(frameC.width, frameC.height);
    silhouetteServer.setName("silhouetteServer");
    warp.setup();
    warp.setTargetRect(ofRectangle(0, 0, kinectWidth, kinectHeight));
    warp.load("warpSaveData.xml");
    warp.enableKeyboardShortcuts();
    warp.setSourceRect(ofRectangle(0, 0, frameC.width, frameC.height));
    trueWarp = warp;
    
    //setup osc
    sender.setup("localhost", 50100);
    
    //setup gui
    kinectGroup.setName("kinect");
    kinectGroup.add(kinectX.set("x", 0, -3000, 3000));
    kinectGroup.add(kinectY.set("y", 0, -3000, 3000));
    kinectGroup.add(kinectFovW.set("fovW", 58.5, 68.5, 48.5));
    
    scopeGroup.setName("scope");
    scopeGroup.add(scopeWidth.set("width", 0, 0, 7000));
    scopeGroup.add(scopeHeight.set("height", 0, 0, 4000));
    scopeGroup.add(scopeDepth.set("depth", 0, 0, 10000));
    
    wallGroup.setName("wall");
    wallGroup.add(wallX.set("x", 0, -3500, 3500));
    wallGroup.add(wallZ.set("z", 0, 0, 10000));
    wallGroup.add(wallWidth.set("width", 3500, 10, 7000));
    
    pointCloudGroup.setName("pointCloud");
    pointCloudGroup.add(bDrawPointCloud.set("bDraw", false));
    pointCloudGroup.add(pointCloudRoll.set("roll", 0, -45, 45));
    pointCloudGroup.add(pointCloudPitch.set("pitch", 0, -45, 45));
    pointCloudGroup.add(pointCloudYaw.set("yaw", 0, -45, 45));
    pointCloudGroup.add(pointCloudDensityReferenceDistance.set("densityReferenceDistance", 10000, 0, 10000));
    pointCloudGroup.add(pointCloudSkip.set("skip", 1, 1, 100));

    personGuiGroup.setName("person");
    personGuiGroup.add(bFindPerson.set("bFind", false));
    personGuiGroup.add(minPointsNumForAPersonSeed.set("minPointsNumForAPersonSeed", 1000, 1, 30000));
    personGuiGroup.add(minJoinDist.set("minJoinDistance", 250, 0, 500));
    
    silhouetteGroup.setName("silhouette");
    silhouetteGroup.add(bFindSilhouette.set("bFind", false));
    silhouetteGroup.add(bDrawSilhouette.set("bDraw", false));
    silhouetteGroup.add(silhouetteImageSkip.set("imageSkip", 1, 1, 10));
    silhouetteGroup.add(silhouetteGapX.set("gapX", 0, -100, 100));
    silhouetteGroup.add(silhouetteGapY.set("gapY", 0, -100, 100));
    
    oscGroup.setName("osc");
    oscGroup.add(host.set("host", "localhost"));
    
    gui.setup();
    gui.add(kinectGroup);
    gui.add(scopeGroup);
    gui.add(wallGroup);
    gui.add(pointCloudGroup);
    gui.add(personGuiGroup);
    gui.add(silhouetteGroup);
    gui.add(oscGroup);
    gui.loadFromFile("guiSaveData.xml");
    
    //load windowRect
    tool.loadWindowRect();
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofSetWindowTitle("person | " + ofToString(round(ofGetFrameRate())) + " | " + ofToString(people.size()));
    
    if (mode == 0) return;
    
    //update osc
    string hostString = host;
    if (hostString != sender.getHost()) sender.setup(hostString, 50100);
    
    //update kinect
    kinect.update();
    ofPoint depthes[kinectHeight][kinectWidth];
    for (int y = 0; y < kinectHeight; y++) {
        for (int x = 0; x < kinectWidth; x++) {
            depthes[y][x] = kinect.getWorldCoordinateAt(x, y);
        }
    }
    
    //update kinectFov
    kinectFovH = atan(tan(kinectFovW * DEG_TO_RAD / 2.) * (kinectHeight / (float)kinectWidth)) * RAD_TO_DEG * 2;
    
    //set scope
    scope.set(scopeWidth, scopeHeight, scopeDepth);
    //kinectY = -scopeHeight / 2.;
    kinectZ = scopeDepth / 2.;
    
    //set wall
    wallHeight = wallWidth * (frameC.height / frameC.width);
    wallDepth = wallHeight * (frameR.width / frameR.height);
    wallY = -scopeHeight / 2. + wallHeight / 2.;
    
    //set pointsFull
    pointsFull.clear();
    for (int y = 0; y < kinectHeight; y++) {
        for (int x = 0; x < kinectWidth; x++) {
            if ((int)ofRandom(pointCloudSkip) == 0 && (int)ofRandom(pow(pointCloudDensityReferenceDistance / kinect.getDistanceAt(x, y), 2)) == 0)
                pointsFull.push_back(math.rotatedPoint(depthes[y][x], 180 + pointCloudRoll, 180 + pointCloudPitch, pointCloudYaw) + ofPoint(kinectX, kinectY, kinectZ));
        }
    }
    
    //set pointCloud
    if (bDrawPointCloud) {
        pointCloud.clear();
        int i = 0;
        for (int y = 0; y < kinectHeight; y++) {
            for (int x = 0; x < kinectWidth; x++) {
                if ((int)ofRandom(pointCloudSkip) == 0 && (int)ofRandom(pow(pointCloudDensityReferenceDistance / kinect.getDistanceAt(x, y), 2)) == 0) {
                    pointCloud.addVertex(math.rotatedPoint(depthes[y][x], 180 + pointCloudRoll, 180 + pointCloudPitch, pointCloudYaw) + ofPoint(kinectX, kinectY, kinectZ));
                    if (math.isInBox(pointCloud.getVertex(i), scope)) pointCloud.addColor(ofColor(0, 255, 0));
                    else pointCloud.addColor(ofColor(255, 0, 0));
                    i++;
                }
            }
        }
    }
    
    //update person
    objects.clear();
    people.clear();
    if (bFindPerson) {
        points.clear();
        for (int i = 0; i < pointsFull.size(); i++) {
            if (math.isInBox(pointsFull[i], scope)) points.push_back(pointsFull[i]);
        }
        if (points.size() >= 2) {
            //nearJoin
            pointGroups.clear();
            pointGroups.push_back(vector<ofPoint>());
            pointGroups.back().push_back(points.front());
            for (int i = 1; i < points.size(); i++) {
                bool bNear;
                for (int j = 0; j < pointGroups.size(); j++) {
                    bNear = false;
                    for (int k = 0; k < pointGroups[j].size(); k++) {
                        if (ofDist(points[i], pointGroups[j][k]) <= minJoinDist) {
                            bNear = true;
                            break;
                        }
                    }
                    if (bNear) {
                        pointGroups[j].push_back(points[i]);
                        break;
                    }
                }
                if (!bNear) {
                    pointGroups.push_back(vector<ofPoint>());
                    pointGroups.back().push_back(points[i]);
                }
            }
            //minFilter
            minPointsNumForAPerson = minPointsNumForAPersonSeed / pointCloudSkip * pow(1000. / pointCloudDensityReferenceDistance, 2);
            int i = 0;
            while (i < pointGroups.size()) {
                if (pointGroups[i].size() < minPointsNumForAPerson) pointGroups.erase(pointGroups.begin() + i);
                else i++;
            }
            //set objects
            for (int i = 0; i < pointGroups.size(); i++) {
                objects.push_back(math.wrapBox(pointGroups[i]));
            }
        }
        
        //set people
        for (int i = 0; i < objects.size(); i++) {
            people.push_back(ofPoint(objects[i].getX(), 0, objects[i].getZ()));
        }
    }
    //send people
    message.clear();
    message.setAddress("person");
    message.addIntArg(people.size());
    for (int i = 0; i < people.size(); i++) {
        message.addFloatArg(ofMap(people[i].x, wallX - wallWidth / 2., wallX + wallWidth / 2., 0, 1));
        message.addFloatArg(ofMap(people[i].z, scopeDepth / 2. - wallZ, scopeDepth / 2. - wallZ + wallDepth, 0, 1));
    }
    sender.sendMessage(message);
    
    if (bFindSilhouette) {
        //update silhouetteTexture
        int width = kinectWidth / silhouetteImageSkip;
        int height = kinectHeight / silhouetteImageSkip;
        unsigned char * silhouettePixels = new unsigned char[width * height * 3];
        int i = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (math.isInBox(math.rotatedPoint(depthes[(int)(y * silhouetteImageSkip)][(int)(x * silhouetteImageSkip)], 180 + pointCloudRoll, 180 + pointCloudPitch, pointCloudYaw) + ofPoint(kinectX, kinectY, kinectZ), scope)) {
                    silhouettePixels[i * 3] = 255;
                    silhouettePixels[i * 3 + 1] = 255;
                    silhouettePixels[i * 3 + 2] = 255;
                } else {
                    silhouettePixels[i * 3] = 0;
                    silhouettePixels[i * 3 + 1] = 0;
                    silhouettePixels[i * 3 + 2] = 0;
                }
                i++;
            }
        }
        silhouetteTexture.allocate(width, height, GL_RGB);
        silhouetteTexture.loadData(silhouettePixels, width, height, GL_RGB);
        delete [] silhouettePixels;
        
        //update warp
        /*for (int i = 0; i < 4; i++) {
            wallPoints[i] = ofPoint(wallX, wallY, scopeDepth / 2. - wallZ);
        }
        wallPoints[0] += ofPoint(-wallWidth, wallHeight) / 2.;
        wallPoints[1] += ofPoint(wallWidth, wallHeight) / 2.;
        wallPoints[2] += ofPoint(wallWidth, -wallHeight) / 2.;
        wallPoints[3] += ofPoint(-wallWidth, -wallHeight) / 2.;
        for (int i = 0; i < 4; i++) {
            ofPoint point = wallPoints[i] - ofPoint(kinectX, kinectY, kinectZ);
            point = math.disRotatedPoint(point, 180 + pointCloudRoll, 180 + pointCloudPitch, pointCloudYaw);
            float x = ofMap(atan2(point.x, point.z) * RAD_TO_DEG, -kinectFovW / 2., kinectFovW / 2., 0, kinectWidth);
            float y = ofMap(atan2(point.y, point.z) * RAD_TO_DEG, -kinectFovH / 2., kinectFovH / 2., 0, kinectHeight);
            wallPointsOnKinect[i] = ofPoint(x, y);
            warp.setCorner(wallPointsOnKinect[i], i);
        }*/
    }
    
    //update silhouetteFbo
    silhouetteFbo.begin();
    ofClear(0);
    
    if (bFindSilhouette) {
        ofSetColor(0);
        ofDrawRectangle(0, 0, silhouetteFbo.getWidth(), silhouetteFbo.getHeight());
        
        ofPushMatrix();
        ofTranslate(silhouetteGapX, silhouetteGapY);
        ofMultMatrix(trueWarp.getMatrixInverse());
        ofSetColor(255);
        silhouetteTexture.draw(0, 0, kinectWidth, kinectHeight);
        ofPopMatrix();
    }
    
    silhouetteFbo.end();
    silhouetteServer.publishTexture(&silhouetteFbo.getTexture());
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    if (mode == 0) {
        //draw alart
        ofSetColor(255, 0, 0);
        ofDrawRectangle(ofGetWindowRect());
    } else if (mode == 1) {
        float radius = cbrt(scope.getWidth() * scope.getHeight() * scope.getDepth()) / 90.;
        
        ofEnableDepthTest();
        easyCam.begin();
        
        ofDrawAxis(256);
        
        //draw kinect
        ofSetColor(255, 255, 0);
        ofDrawSphere(kinectX, kinectY, kinectZ, radius);
        
        //draw pointCloud
        if (bDrawPointCloud) pointCloud.draw();
        
        ofPushStyle();
        ofNoFill();
        
        //draw scope
        ofSetColor(255);
        ofDrawBox(scope.getPosition(), scope.getWidth(), scope.getHeight(), scope.getDepth());
        
        //draw objects
        for (int i = 0; i < objects.size(); i++) {
            ofDrawBox(objects[i].getPosition(), objects[i].getWidth(), objects[i].getHeight(), objects[i].getDepth());
        }
        
        ofPopStyle();
        
        //draw people
        for (int i = 0; i < people.size(); i++) {
            ofColor color;
            color.setHsb((int)ofMap(i, 0, 6, 0, 256) % 256, 255, 255);
            ofSetColor(color);
            ofDrawSphere(people[i], 10);
        }
        ofPushStyle();
        ofSetColor(127);
        ofNoFill();
        for (int i = 0; i < people.size(); i++) {
            ofDrawSphere(people[i], minJoinDist);
        }
        ofPopStyle();
        
        //draw wall
        ofPushMatrix();
        ofTranslate(ofPoint(wallX, wallY, scopeDepth / 2. - wallZ));
        ofSetColor(0, 0, 255);
        ofDrawRectangle(-wallWidth / 2., -wallHeight / 2., wallWidth, wallHeight);
        ofTranslate(wallWidth / 2., 0);
        ofRotateYDeg(270);
        ofSetColor(0, 255, 255);
        ofDrawRectangle(0, -wallHeight / 2., wallDepth, wallHeight);
        ofPopMatrix();
        
        easyCam.end();
        ofDisableDepthTest();
        
        //draw silhouetteTexture
        if (bFindSilhouette && bDrawSilhouette) {
            ofPushMatrix();
            ofSetColor(255);
            ofTranslate(ofGetWidth() - kinectWidth, ofGetHeight() - kinectHeight);
            kinect.draw(0, 0);
            silhouetteTexture.draw(0, 0, kinectWidth, kinectHeight);
            ofPopMatrix();
        }
        
        //draw gui
        gui.draw();
    } else {
        /*ofPushMatrix();
        translator.reset();
        translator.smartFit(ofRectangle(0, 0, silhouetteFbo.getWidth(), silhouetteFbo.getHeight()), ofGetWindowRect());
        
        //draw silhouetteFbo
        ofSetColor(255);
        silhouetteFbo.draw(0, 0);
        
        //draw frame
        ofPushStyle();
        ofNoFill();
        ofSetColor(255);
        ofDrawRectangle(0, 0, silhouetteFbo.getWidth(), silhouetteFbo.getHeight());
        ofPopStyle();
        
        ofPopMatrix();*/
            
        ofImage colorImage;
        colorImage.allocate(kinectWidth, kinectHeight, OF_IMAGE_COLOR_ALPHA);
        for (int y = 0; y < kinectHeight; y++) {
            for (int x = 0; x < kinectWidth; x++) {
                colorImage.setColor(x, y, ofColor(kinect.getWorldCoordinateAt(x, y).z));
            }
        }
        
        ofPushMatrix();
        translator.reset();
        translator.smartFit(ofRectangle(0, 0, kinectWidth, kinectHeight), ofGetWindowRect());
        ofSetColor(255);
        colorImage.draw(0, 0);
        ofPopMatrix();
        ofSetColor(255, 0, 255);
        warp.draw();
        
        for (int i = 0; i < 4; i++) {
            trueWarp.dstPoints[i] = translator.getDisTranslatedPosition(warp.dstPoints[i]);
        }
    }
}

//--------------------------------------------------------------
void ofApp::exit() {

    //save kinectIndex
    xml.clear();
    xml.setValue("kinectIndex", kinectIndex);
    xml.save("kinectIndexSaveData.xml");
    
    //save warp
    warp.save("warpSaveData.xml");

    //save gui
    gui.saveToFile("guiSaveData.xml");
    
    //save windowRect
    tool.saveWindowRect();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

    //switch kinect
    if (key == ' ') {
        if (mode == 0) {
            kinectIndex = 0;
        } else {
            kinectIndex++;
            if (kinectIndex >= kinect.numTotalDevices()) kinectIndex = 0;
        }
        kinect.close();
        kinect.init();
        mode = kinect.open(kinectIndex);
        kinect.setRegistration(true);
        kinectWidth = kinect.width, kinectHeight = kinect.height;
    }
    
    //switch mode
    if (key == OF_KEY_RETURN) {
        mode++;
        if (mode >= 3) mode = 1;
        if (mode == 1) easyCam.enableMouseInput();
        if (mode == 2) easyCam.disableMouseInput();
    }
    
    //reset warp
    if (mode == 2 && (key == 'r' || key == 'R')) warp.setTargetRect(ofRectangle(ofGetWidth() / 3., ofGetHeight() / 3., ofGetWidth() / 3., ofGetHeight() / 3.));
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
    
    if (ofGetFrameNum() == 0) return;
    
    ofPushMatrix();
    translator.reset();
    translator.smartFit(ofRectangle(0, 0, kinectWidth, kinectHeight), ofGetWindowRect());
    for (int i = 0; i < 4; i++) {
        warp.dstPoints[i] = translator.getTranslatedPosition(trueWarp.dstPoints[i]);
    }
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
