#pragma once

#include "ofMain.h"

#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxSyphon.h"
#include "ofxGui.h"
#include "ofxQuadWarp.h"
#include "ofxOsc.h"
#include "ofxTskokmtMath.h"
#include "ofxTskokmtTool.h"
#include "ofxTskokmtTranslator.h"

class ofApp: public ofBaseApp {

public:
    void setup();
    void update();
    void draw();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void exit();
		
private:
    //mode
    int mode = 0;
    
    //frame
    ofRectangle frameC;
    ofRectangle frameL;
    ofRectangle frameR;
    ofRectangle frame;
    
    //kinect
    ofxKinect kinect;
    int kinectIndex = 0;
    int kinectWidth;
    int kinectHeight;
    ofParameter<int> kinectX;
    ofParameter<int> kinectY;
    int kinectZ;
    ofParameter<float> kinectFovW;
    ofParameterGroup kinectGroup;
    
    //camera
    ofEasyCam easyCam;
    
    //scope
    ofParameter<int> scopeWidth;
    ofParameter<int> scopeHeight;
    ofParameter<int> scopeDepth;
    ofBoxPrimitive scope;
    ofParameterGroup scopeGroup;
    
    //wall
    ofParameter<int> wallX;
    int wallY;
    ofParameter<int> wallZ;
    ofParameter<float> wallWidth;
    float wallHeight;
    float wallDepth;
    ofParameterGroup wallGroup;
    
    //pointCloud
    ofMesh pointCloud;
    ofMesh pointCloudZ;
    ofMesh pointCloudX;
    vector<ofPoint> points;
    vector<ofPoint> pointsFull;
    ofParameter<float> pointCloudRoll;
    ofParameter<float> pointCloudPitch;
    ofParameter<float> pointCloudYaw;
    ofParameter<int> pointCloudSkip;
    ofParameter<int> pointCloudDensityReferenceDistance;
    ofParameter<bool> bDrawPointCloud;
    ofParameterGroup pointCloudGroup;

    //person
    ofParameter<bool> bFindPerson;
    vector<ofBoxPrimitive> objects;
    vector<ofPoint> people;
    vector<vector<ofPoint>> pointGroups;
    ofParameter<int> minPointsNumForAPersonSeed;
    int minPointsNumForAPerson;
    ofParameter<float> minJoinDist;
    ofParameterGroup personGuiGroup;
    
    //silhouette
    ofParameter<bool> bFindSilhouette;
    ofParameter<bool> bDrawSilhouette;
    ofTexture silhouetteTexture;
    ofFbo silhouetteFbo;
    ofxSyphonServer silhouetteServer;
    ofPoint wallPoints[4];
    ofPoint wallPointsOnKinect[4];
    float kinectFovH;
    ofxQuadWarp warp;
    ofxQuadWarp trueWarp;
    ofParameter<float> silhouetteImageSkip;
    ofParameter<float> silhouetteGapX;
    ofParameter<float> silhouetteGapY;
    ofParameterGroup silhouetteGroup;
    ofxCv::ContourFinder contourFinder;
    
    //osc
    ofxOscSender sender;
    ofxOscMessage message;
    ofParameter<string> host;
    ofParameterGroup oscGroup;
    
    //gui
    ofxPanel gui;
    
    //xml
    ofxXmlSettings xml;
    
    //math
    ofxTskokmtMath math;
    
    //tool
    ofxTskokmtTool tool;
    
    //translator
    ofxTskokmtTranslator translator;
};
