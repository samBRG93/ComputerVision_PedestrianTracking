// test.cpp : Defines the entry point for the console application.
//


#include "stdafx.h"



#include <opencv2\opencv.hpp>
/*
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/core/ocl.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2\features2d.hpp>
*/
#include <tuple>
#include <iostream>

#include "functions.h"
#include <list>
#include <iostream>



using namespace cv;
using namespace std;



#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

int main()
{
	
	// lettura video in ingresso 
	string filename = "video.avi" ;
	VideoCapture cap(filename);
	
	// Exit if video is not opened
	if (!cap.isOpened())
	{
		cout << "Could not read video file" << endl;
		return 1;

	} 
	// definizione Mat delle immagini 
	Mat frame;
	Mat MaskMOG2;
	Mat Mask(1916, 1409, CV_8UC1, Scalar(0));
	Mat ROI(1916, 1409, CV_8U, Scalar(0));

	//MOG2 settings
	int history = 500; //500;
	float varThreshold = 80; //40;
	bool bShadowDetection = true;
	float learning_rate = 0.1;

	//definizione di MOG2 : backgrown subtractor 
	Ptr<BackgroundSubtractorMOG2> pMOG2; 
	pMOG2 = createBackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
	cap >> frame; 
	
	
	//definizione posizione dei rettangoli
	int x11 = 525, x12 = 400, y11 = 256, y12 = 300;
	int x21 = 535, x22 = 180, y21 = 1385, y22 = 150;
	int x31 = 15, x32 = 176, y31 = 900, y32 = 250; 
	int x41 = 1100, x42 = 300, y41 = 800, y42 = 2 * 175;
	

	// definizione dei rettangoli per la ROI 
	Rect RecS(x11, y11, x12, y12);
	Rect RecN(x21, y21, x22, y22 - 10);
	Rect RecW(x31, y31, x32, y32+55 - 85);
	Rect RecE(x41, y41, x42, y42);;

	Mask(RecN).setTo(Scalar(255));
	Mask(RecS).setTo(Scalar(255));
	Mask(RecE).setTo(Scalar(255));
	Mask(RecW).setTo(Scalar(255));
	
	// definizione dei rettangoli per  controlli 
	
	Rect RecNn(x11 - 25, y11, 500, 20); // 25
	Rect RecNs(x11 - 25, y11 - 25 + y12, 500, 20); //25
	Rect RecSn(x21 - x22, y21, 500, 20); //25
	Rect RecSs(x21 - x22, y21 -25-10 + y22 , 500, 20); //25
	Rect RecEw(x41, y41, 20, 500); //25
	Rect RecEe(x41 + x42 - 25, y41 - 25, 20, 500);//25
	Rect RecWw(x31, y31 - 25, 20, 500); //25
	Rect RecWe(x32 +20-30, y31 - 25, 20, 500); // 25
	
	int Window = 50;
	int StartFlag = 0;
	float maxTargetArea = 850000;

	Scalar color(0, 0, 0);
	Scalar red(0, 0, 255);
	Scalar green(0, 255, 0);
	Scalar blue(255, 0, 0);
	vector<Mat> channels;
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;
	vector<vector<Point2d>> edges;
	vector<Point2d> TriangleBaricenterPrevious;
	vector<Ref> Correspondances;
	vector<vector<Ref>> FramesCorrespondances; 
	vector<vector<int>> PointingPrev;
	vector<Candidate> Candidates;
	tuple<Point2d, Point2d, Point2d, Point2d> incremental = { 
		Point2d(0,0),Point2d(0,0),Point2d(0,0),Point2d(0,0) };
	
	// setting for video output 
	/*
	int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	VideoWriter video("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(frame_width, frame_height));
	*/
	// per il caso particolare 
    cap >> frame; cap >> frame; 

	while (true)
	{
		// salto alcuni frames per aumentare la differenza tra un frame e l'altro 
		cap >> frame; 
	    
		// sottraggo il background 
		pMOG2->apply(frame, MaskMOG2);
		MaskMOG2 = Filtering(MaskMOG2);
		MaskMOG2.copyTo(ROI, Mask);

		// trova i contorni nella ROI  
		findContours(ROI.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

		// definisce i triangoli che servono per fare un controllo dell'area degli oggetti 
		size_t count = contours.size();
		for (int i = 0; i<count; i++)
		{
			vector<Point2d> triangle;
			minEnclosingTriangle(contours[i],triangle);	
			int AreaT = norm(Mat(triangle[0]), Mat(triangle[1]))*norm(Mat(triangle[0]), 
				Mat(triangle[2]))*norm(Mat(triangle[2]), Mat(triangle[1]));

			if (AreaT < maxTargetArea) {
				edges.push_back(triangle);
			}
		}
		// aggiornati ad ogni frame 
		size_t countx = edges.size();
		vector<Point2d> T;
		vector<int> AreaTriangle;
		vector<Point2d> TriangleBaricenter;

		// definisce le bounding box: grandezze,posizione 
		vector<Rect> boundRect;
		int SizeB = 50; 
		
		for (int i = 0;i < TriangleBaricenterPrevious.size();i++) {
			if ((RecN.contains(TriangleBaricenterPrevious[i]) || RecS.contains(TriangleBaricenterPrevious[i]) ||
				RecE.contains(TriangleBaricenterPrevious[i]) || RecW.contains(TriangleBaricenterPrevious[i]))) {
				boundRect.push_back(Rect(TriangleBaricenterPrevious[i].x - SizeB/2, TriangleBaricenterPrevious[i].y - SizeB/2,SizeB,SizeB));
				rectangle(frame, boundRect[i], green, 3);
			}
		}

		// definisce i triangoli, calcola baricentro e una misura rappresentativa dell'area 
		for (int i = 0; i < countx; i++) {
					T = edges[i];
					AreaTriangle.push_back(norm(Mat(T[0]), Mat(T[1]))*norm(Mat(T[0]), Mat(T[2]))*norm(Mat(T[2]), Mat(T[1])));
					TriangleBaricenter.push_back((T[0]+T[1]+T[2]) / 3);	
		
					for (int j = 0; j < 3; j++) {
						if (AreaTriangle[i] > 35000) { color = red; }
						else { color = green; }
					}			
				//per stampare i triangoli che fanno riferimento al controllo sull'area
				//line(frame, T[j], T[(j + 1) % 3], color, 3); 
		}
		

		// trova le posizioni di corrispondenza tra gli ultimi due frames
		Correspondances = FindCenterCorrespondance(boundRect,TriangleBaricenter, TriangleBaricenterPrevious);
		if (Correspondances.size() != 0) {
			for(int i=0;i<Correspondances.size();i++)
				circle(frame, Correspondances[i].Cur, 5, red, 3);
		}
	
		// considero una certa finestra per le acquisizioni
		if (FramesCorrespondances.size() > Window ) {
			FramesCorrespondances.erase(FramesCorrespondances.begin(), FramesCorrespondances.begin() + 1);
		}
		
		// inserimento nell'ultima posizione della finesta 
		if (Correspondances.size() != 0) {
			FramesCorrespondances.push_back(Correspondances);
		}
		
		
		// print dei bacentri  
		/*
		for (int i = 0;i<Correspondances.size();i++) {
			circle(frame, Correspondances[i].Prev, 5, red);
			circle(frame, Correspondances[i].Cur, 5, red,3);
		}
		*/
		// aggiornamento dei baricentri e dei rettangoli
		TriangleBaricenterPrevious.clear();
		TriangleBaricenterPrevious = TriangleBaricenter; 
		
		//pulisci buffers
		if (edges.size() != 0) {
			edges.clear();
			AreaTriangle.clear();
			TriangleBaricenter.clear();
		}
		
		// inserimento di elementi nei candidati in base alle corrispondenzee nei frames 
		InsertCandidates(Candidates, FramesCorrespondances);
		
		// conteggio dei passaggi 
		tuple<Point2d, Point2d, Point2d, Point2d> Temp = PassageCounter(Candidates);
		
		get<0>(incremental) = get<0>(incremental) + get<0>(Temp);
		get<1>(incremental) = get<1>(incremental) + get<1>(Temp);
		get<2>(incremental) = get<2>(incremental) + get<2>(Temp);
		get<3>(incremental) = get<3>(incremental) + get<3>(Temp);

		// stampa conteggi 
		cout << "--------------------------------------------------" << endl;
		cout << "stampa conteggi Nord: " << get<0>(incremental) << endl;
		cout << "stampa conteggi West: " << get<1>(incremental) << endl;
		cout << "stampa conteggi Sud: " << get<2>(incremental) << endl;
		cout << "stampa conteggi Est: " << get<3>(incremental) << endl;
		cout << "--------------------------------------------------" << endl;
		cout << endl;
	    
		int N_loop = 1;  // 1 se voglio mostrare il video 2 se voglio salvarlo 
		for (int i = 0;i < N_loop;i++) {
			
			cap >> frame;

			for(int j=0;j<boundRect.size();j++)
				rectangle(frame, boundRect[j], green, 3);
			
			// mostro i rettangoli della ROI 
			rectangle(frame, RecN, Scalar(0, 0, 255), 3, 8, 0);
			rectangle(frame, RecS, Scalar(0, 0, 255), 3, 8, 0);
			rectangle(frame, RecW, Scalar(0, 0, 255), 3, 8, 0);
			rectangle(frame, RecE, Scalar(0, 0, 255), 3, 8, 0);

			// per mostrare i rettangoli per il controllo  
		/*
			rectangle(frame, RecWw, Scalar(0, 255, 0), 3, 8, 0);
			rectangle(frame, RecWe, Scalar(0, 255, 0), 3, 8, 0);
			rectangle(frame, RecSs, Scalar(0,255,0), 3, 8, 0);
			rectangle(frame, RecSn, Scalar(0,255,0), 3, 8, 0);
			rectangle(frame, RecNs, Scalar(0,255,0), 3, 8, 0);
			rectangle(frame, RecNn, Scalar(0,255,0), 3, 8, 0);
			rectangle(frame, RecEe, Scalar(0,255,0), 3, 8, 0);
			rectangle(frame, RecEw, Scalar(0,255,0), 3, 8, 0);
		*/
			// stampa dell' ultima posizione conosciuta dei candidati 
			for (int i = 0;i<Candidates.size();i++)
				circle(frame, Candidates[i].LastKnown, 7, blue, 3);

			// scrivi risultati sui frame 
			string word;
			stringstream Convert;
			Convert.clear();

			Convert << int(get<0>(incremental).x);
			word = "OUT: " + Convert.str();
			putText(frame, word, cvPoint(100, 300), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");
			Convert << int(get<0>(incremental).y);
			word = "IN: " + Convert.str();
			putText(frame, word, cvPoint(100, 400), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");

			Convert << int(get<1>(incremental).x);
			word = "OUT: " + Convert.str();
			putText(frame, word, cvPoint(10, 1200), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");
			Convert << int(get<1>(incremental).y);
			word = "IN: " + Convert.str();
			putText(frame, word, cvPoint(10, 1300), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");

			Convert << int(get<2>(incremental).y);
			word = "OUT: " + Convert.str();
			putText(frame, word, cvPoint(750, 1425), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");
			Convert << int(get<2>(incremental).x);
			word = "IN: " + Convert.str();
			putText(frame, word, cvPoint(750, 1525), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");

			Convert << int(get<3>(incremental).y);
			word = "OUT: " + Convert.str();
			putText(frame, word, cvPoint(1000, 650), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");

			Convert << int(get<3>(incremental).x);
			word = "IN: " + Convert.str();
			putText(frame, word, cvPoint(1000, 750), FONT_HERSHEY_SIMPLEX, 2, cvScalar(0, 255, 0), 3, CV_AA);
			Convert.str("");

			// show frames
			
			namedWindow("Frame", CV_WINDOW_NORMAL);  resizeWindow("frame", 500, 600);
			imshow("Frame", frame);
			
			// per vedere il backgrownd
			/*
			namedWindow("mask", CV_WINDOW_NORMAL); resizeWindow("mask", 500, 600);
			imshow("mask", MaskMOG2);
			*/
			// per visualizzare solo la region of interest 
			/*
			namedWindow("ROI", CV_WINDOW_NORMAL);
			imshow("ROI", ROI);
			*/


			// Write the frame into the file 'outcpp.avi'
			//video.write(frame);

		}
			
		// Exit if ESC pressed.
		int k = waitKey(1);
		if (k == 27)
		{
			break;
		}
		
	
		// pulisci buffers
		Correspondances.clear();
		StartFlag++;
		boundRect.clear();
		waitKey(1);

	}
	//video.release();
	cap.release();
	//destroyAllWindows();

    return 0;

}

