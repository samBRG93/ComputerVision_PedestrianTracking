#include "stdafx.h"
#include "functions.h"

int x11 = 525, x12 = 400, y11 = 256, y12 = 300;
int x21 = 535, x22 = 180, y21 = 1385, y22 = 150;
int x31 = 15, x32 = 176, y31 = 900, y32 = 250; //50
int x41 = 1100, x42 = 300, y41 = 800, y42 = 2 * 175;


Rect RecS(x11, y11, x12, y12);
Rect RecN(x21, y21, x22, y22 - 10);
Rect RecW(x31, y31, x32, y32 + 55 - 85);
Rect RecE(x41, y41, x42, y42);;


Rect RecNn(x11 - 25, y11, 500, 20); // 25
Rect RecNs(x11 - 25, y11 - 25 + y12, 500, 20); //25
Rect RecSn(x21 - x22, y21, 500, 20); //25
Rect RecSs(x21 - x22, y21 - 25 - 10 + y22, 500, 20); //25
Rect RecEw(x41, y41, 20, 500); //25
Rect RecEe(x41 + x42 - 25, y41 - 25, 20, 500);//25
Rect RecWw(x31, y31 - 25, 20, 500); //25
Rect RecWe(x32 + 20 - 30, y31 - 25, 20, 500); // 25

ostream& operator<<(ostream& os, const Ref& a) {
	
	os << "Prev: " << a.Prev << endl;
	os << "Cur: " << a.Cur << endl;
	os << "Ste: " << a.Step << endl;
	os << "Angle: " << a.Angle << endl;

	return os;
}

ostream& operator<<(ostream& os, const Candidate& a) {

	os << "Entry: " << a.Entry << endl;
	os << "Last known" << a.LastKnown << endl;
	os << "Exit: " << a.Exit << endl;
	for (int i = 0;i < a.Angles.size();i++) {
		os << "Angles i=" << i << ": " << a.Angles[i]<<"----" << "Steps i=" << i << ": " << a.Steps[i] << endl;
	}
	
	os << endl;
	os << "Angle Std: " << a.StdAngles << endl;
	os << "Step Std: " << a.StdSteps << endl;
	os << "TimeLapse: " << a.TimeLapse << endl;
	os << "Angle Size: " << a.Angles.size() << endl;
	os << "difference of sizes" << (int(a.TimeLapse) - int(a.Angles.size())) << endl;
	
	return os;
}

void Test() {
	cout << "Test" << endl;
}


Mat Filtering(Mat MaskMOG2) {

	medianBlur(MaskMOG2(RecN), MaskMOG2((RecN)), 3); 
	medianBlur(MaskMOG2(RecS), MaskMOG2((RecS)), 3);
	medianBlur(MaskMOG2(RecE), MaskMOG2((RecE)), 3);
	medianBlur(MaskMOG2(RecW), MaskMOG2((RecW)), 3);
	dilate(MaskMOG2, MaskMOG2, Mat(), Point(-1, -1), 3, 0, 0);
	erode(MaskMOG2, MaskMOG2, Mat(), Point(-1, -1), 4, 0, 0); 
	dilate(MaskMOG2, MaskMOG2, Mat(), Point(-1, -1), 5, 0, 0); 
	blur(MaskMOG2, MaskMOG2, Size(3, 3));
	threshold(MaskMOG2, MaskMOG2, 25, 255, THRESH_BINARY); 
	return MaskMOG2;
}

float EuclidianDistance(Point2d a, Point2d b) {
	
	Point2d Res = a - b; 
	return hypot(Res.x,Res.y);
}

vector<Ref> FindCenterCorrespondance(vector<Rect> boundRect,vector<Point2d> TriangleBaricenterCurrent,
										vector<Point2d> TriangleBaricenterPrevious) 
{
	float Dist;
	float CompDist;
	vector<Point2d> Correspondances;
	vector<Point2d> CurrentPositions;
	vector<float> Distances;
	vector<Ref> TotalCorrespondances;
	vector<int> posi; 
	int PosScal = 0;
	float min = 999999999;
	int pos = 99999999;

	if (TriangleBaricenterCurrent.size() != 0 && TriangleBaricenterPrevious.size() != 0) {
		
		for (int i = 0;i < TriangleBaricenterPrevious.size();i++) {
			for(int j = 0;j < TriangleBaricenterCurrent.size();j++) {	
			
				if (boundRect[i].contains(TriangleBaricenterCurrent[j]) &&
					EuclidianDistance(TriangleBaricenterCurrent[j], TriangleBaricenterPrevious[i])<min) {
					min = EuclidianDistance(TriangleBaricenterCurrent[j], TriangleBaricenterPrevious[i]);
					pos = j; 
				}	
				
			}
			// se non è in nessuna delle bounding box
			
			if (pos == 99999999) {
				TotalCorrespondances.push_back(Ref(TriangleBaricenterPrevious[i], 
					TriangleBaricenterPrevious[i]));
				
			}
			else {

				TotalCorrespondances.push_back(Ref(TriangleBaricenterPrevious[i], 
					TriangleBaricenterCurrent[pos]));
			}
			min = 999999999;
			pos = 99999999;
		}
		
	}	
	return TotalCorrespondances;
}

void InsertCandidates(vector<Candidate>& Candidates, vector<vector<Ref>> FramesCorrespondances) {
	
	if (FramesCorrespondances.size() != 0) {
		vector<Ref> Correspondances = FramesCorrespondances[FramesCorrespondances.size() - 1];
		if (FramesCorrespondances.size() == 1) {
			for (int i = 0;i < Correspondances.size();i++) {
				Candidates.push_back(Candidate(Correspondances[i].Cur, NULL, NULL));
			}
		}
		if (FramesCorrespondances.size() >= 2) {
			vector<Ref> PreviousCorrespondances = FramesCorrespondances[FramesCorrespondances.size() - 2];


			// valuta i candidati esistenti 
			for (int i = 0;i < Candidates.size();i++) {
				Candidates[i].InsertProgressFlow(Correspondances,Candidates); // Size_Cand,Last
			}
			// nuovi candidati da inserire, quelli che non avevano corrispondenza non esistevano
			vector<int> counter;
			for (int i = 0;i < Correspondances.size(); i++) {
				counter.push_back(0);
				for (int j = 0;j < PreviousCorrespondances.size();j++) {
					if (Correspondances[i].Prev == PreviousCorrespondances[j].Cur) {
						counter[i]++;
					}
				}
			}
			// counter fornisce la posizione del nuovo condidato da aggiungere ovvero la sua corrispondenza 
			bool AlreadEx = 0; 
			for (int i = 0;i < counter.size();i++) {

				if (counter[i] == 0) {
					for (int j = 0;j < Candidates.size();j++) {
						// valuta se non c'è corrispondena con altri candidati 
						if (Correspondances[i].Cur == Candidates[j].LastKnown) AlreadEx = 1; 
					}
				}
				if (counter[i] == 0 && AlreadEx == 0) {
					Candidates.push_back(Candidate(Correspondances[i].Cur, Correspondances[i].Angle, Correspondances[i].Step));
				}
			}

		}
	
	}
	
}

// funzione per contare i passaggi 
tuple<Point2d, Point2d, Point2d, Point2d> PassageCounter(vector<Candidate>& Candidates) {
	
	float StdA=600, StdS=150;
	bool inside = 0; 
	tuple<Point2d, Point2d, Point2d, Point2d> incremental={Point2d(0,0),Point2d(0,0),Point2d(0,0),Point2d(0,0)};
	
	for (int i = 0;i < Candidates.size();i++) {
		
		// se i candidati non rispettano alcune 
		if (Candidates[i].TimeLapse > 5 && //5
			(int(Candidates[i].TimeLapse)-int(Candidates[i].Angles.size()))>2) { //2
			
			Candidates.erase(Candidates.begin() + i);
			
			cout << Candidates.size() << endl;
			cout << "###############################" << endl;
			cout << "candidato eliminato per differenza in step lapse" << endl;
			cout << "###############################" << endl;
			
		}		
		inside = 0;
		
		if (Candidates.size() > i) { 
			// il numero di step dei candidati deve essere amleno 10, Std = standard deviation 
			if (Candidates[i].Angles.size() > 10 && Candidates[i].StdAngles[0] < StdA && Candidates[i].StdSteps[0] < StdS
				&& (int(Candidates[i].TimeLapse) - int(Candidates[i].Angles.size()))<=3) { 
 				
				if (RecNn.contains(Candidates[i].LastKnown)) { get<0>(incremental).x++; inside = 1; }
				if (RecNs.contains(Candidates[i].LastKnown)) { get<0>(incremental).y++; inside = 1; }
				if (RecWw.contains(Candidates[i].LastKnown)) { get<1>(incremental).x++; inside = 1; }
				if (RecWe.contains(Candidates[i].LastKnown)) { get<1>(incremental).y++; inside = 1; }
				if (RecSn.contains(Candidates[i].LastKnown)) { get<2>(incremental).x++; inside = 1; }
				if (RecSs.contains(Candidates[i].LastKnown)) { get<2>(incremental).y++; inside = 1; }
				if (RecEw.contains(Candidates[i].LastKnown)) { get<3>(incremental).x++; inside = 1; }
				if (RecEe.contains(Candidates[i].LastKnown)) { get<3>(incremental).y++; inside = 1; }

				if (inside == 1) {
					
					cout << Candidates.size() << endl;
					Candidates.erase(Candidates.begin() + i);
					cout << Candidates.size() << endl;
					cout << "###############################" << endl;
					cout << "candidato eliminato: non rispettato inside" << endl;
					cout << "###############################" << endl;
				}
			}

		}
	}
	return incremental;
}

