#pragma once

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

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


using namespace std;
using namespace cv;



float EuclidianDistance(Point2d a, Point2d b);
void Test();

// classe per le referenze, tiene conto dei legami di corrispondenze tra un frame e l'altro 
class Ref {
	public:
		Point2d Prev;
		Point2d Cur;
		float Angle;
		float Step;

		Ref::Ref() {
			Prev = Point2d(0.f, 0.f);
			Cur = Point2d(0.f, 0.f);
			Step = 0.0;
			Angle = 0; 
		}
		Ref::Ref(Point2d _Prev, Point2d _Cur) {
			Prev = _Prev;
			Cur = _Cur;
			Step = EuclidianDistance(_Cur, _Prev);
			Angle = abs(atan2(Cur.y - Prev.y,Cur.x - Prev.x)*(180/3.1415));
		}
		Ref::~Ref() {

		}
	public:
		bool operator==(const Ref& R){
			if (Prev == R.Prev && Cur == R.Cur && Step == R.Step && Angle == R.Angle) {
				return true;
			}
			else {
				return false;
			}
		}
		friend ostream& operator<<(ostream& os, const Ref& a);
		
};

// classe candidato, serve per tenere in conto la storia dei candidati e inserire le corrispondenze relative 
class Candidate{
	public:
		Point2d Entry;
		Point2d Exit;
		Point2d LastKnown;
		vector<float> Angles;
		vector<float> Steps;
		Scalar StdAngles;
		Scalar StdSteps;
		int TimeLapse;

		Candidate::Candidate(Point2d _Entry,float _Angle,float _Step) {
			Entry = _Entry; 
			Exit = Point2d(0.f, 0.f);
			LastKnown = _Entry;
			Angles.push_back(_Angle);
			Steps.push_back(_Step);	
			StdAngles = 0; 
			StdSteps = 0; 
			TimeLapse = 0;
		}
		Candidate::Candidate() {
			Exit = Point2d(0.f, 0.f);
			Entry = Point2d(0.f, 0.f);
			LastKnown = Entry;
			StdAngles = 0;
			StdSteps = 0;
			TimeLapse = 0;
		}
		Candidate::~Candidate() {
			if (Angles.size() > 0) {
				Angles.clear();
				Steps.clear();
			}
		}
		friend ostream& operator<<(ostream& os, const Candidate& a);
		void stampa() {
			cout << "stampa candidato" << endl;
			cout << endl;
			cout << "Entry: " << Entry << endl;
			cout << "Exit: " << Exit << endl;
			cout << "stdAngles: " << StdAngles << endl;
			cout << "stdSteps: " << StdSteps << endl;
		}
		void InsertExit(Point2d _Exit) {
			Exit = _Exit; 
		}
		
		// inseriscol il progesso di candidati durante il movimento 
		void InsertProgressFlow(vector<Ref> _ref, vector<Candidate>& Candidates) { 
			Scalar a;
			TimeLapse++;
			bool update = 0,control = 0,check = 0; 
			float  min = EuclidianDistance(_ref[0].Cur, LastKnown); 
			int pos=0;


			
			// rimuovo i lastknown in eccesso
			vector<int>  Corrisp, Posi;
			int Size = 0; 

			// se ci sono le corrispondenze 
			for (int i = 0;i < Candidates.size();i++) {
				if (LastKnown == Candidates[i].LastKnown) {
					Posi.push_back(i);	
				}
			}
			int size = 999999999, P = 0;
			if (Posi.size() > 1) {
				
				for (int i=0;i < Posi.size();i++) {
					if (Candidates[Posi[i]].Angles.size() < size) {
						size = Candidates[Posi[i]].Angles.size();
						P = Posi[i];
					}
				}
			}
			if (size != 999999999) Candidates.erase(Candidates.begin() + int(P)); 
		
			// da eviare doppioni , da valutare control == 0 
			for (int i = 0;i < _ref.size();i++) {
				
				if (_ref[i].Prev == LastKnown && control == 0) {
					Angles.push_back(_ref[i].Angle);
					Steps.push_back(_ref[i].Step);
					LastKnown = _ref[i].Cur;
					meanStdDev(Angles, a, StdAngles);
					meanStdDev(Steps, a, StdSteps);
					update = 1; 
					control = 1; 
				}	
			}

			// per evitare sovraposizioni di posizioni in caso di flickering 
			if (update == 0 && (int(TimeLapse) - int(Angles.size()) < 2) && (Angles.size() > 4)) { // 2 4 
				
				cout << "###################" << endl;
				cout << "Cerca possibilita' flickering" << endl;
				cout << "###################" << endl;
				
				Rect Bound = Rect(LastKnown.x-200, LastKnown.y-200, 400, 400);
				bool check = 0;
				
				for (int i = 0;i < _ref.size();i++) {
					
					for (int j = 0;j < Candidates.size();j++) {
						if (Candidates[j].LastKnown == _ref[i].Cur) {
							check = 1; 
						}
					}
					cout << "sono uno solo" << endl;
					if (check == 0) {
						if (i == 0 && Bound.contains(_ref[i].Cur)) {
							min = EuclidianDistance(_ref[i].Cur, LastKnown);
							pos = 0;
						}
						if (i != 0 && EuclidianDistance(_ref[i].Cur, LastKnown) < min) {
							pos = i;
						}
					}
					check = 0; 
				}

				if (Bound.contains(_ref[pos].Cur) && check == 0) {

					Angles.push_back(_ref[pos].Angle);
					Steps.push_back(_ref[pos].Step);
					LastKnown = _ref[pos].Cur;
					meanStdDev(Angles, a, StdAngles);
					meanStdDev(Steps, a, StdSteps);
					cout << "ho inserito in: " << LastKnown << endl;
				}
				
				
			}
			
		}
};


Mat Filtering(Mat MaskMOG2);
vector<Ref> FindCenterCorrespondance(vector<Rect> boundRect, vector<Point2d> TriangleBaricenterCurrent,
	vector<Point2d> TriangleBaricenterPrevious);
void InsertCandidates(vector<Candidate>& Candidates, vector<vector<Ref>> FramesCorrespondances);
tuple<Point2d, Point2d, Point2d, Point2d> PassageCounter(vector<Candidate>& Candidates);


#endif // FUNCTIONS_H


