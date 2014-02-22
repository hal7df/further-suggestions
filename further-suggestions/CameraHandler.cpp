#include "cameraHandler.h"
#define TWO_IMAGES
	CameraHandler::CameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, Relay *relay)	
	{
		//This runs once when cameraHandler Class is initilized
		camera->WriteResolution(AxisCamera::kResolution_320x240);
		camera->WriteBrightness(40);

		this->img = new ColorImage(IMAQ_IMAGE_HSL);
		//this->img2 = new ColorImage(IMAQ_IMAGE_HSL);
		this->camera = camera;
		this->m_dsLCD = m_dsLCD;
		this->light = relay;
		this->m_ds = DriverStation::GetInstance();
	}
	
	bool particleSort (ParticleAnalysisReport i, ParticleAnalysisReport j) {return (i.particleArea > j.particleArea);}
	
	
	double CameraHandler::getCenter()
	{
		//m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Test");
		//m_dsLCD->UpdateLCD();
		//Get new camera image
		camera->GetImage(img);
		
		//img.Write("bob2.jpg");  //Cannot work with non-RGB images
		
		//int Wid = img->GetWidth();  //Sanity Check: Check width of image
		//Prints width of image from camera
		//m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,"Width of Image: %d",Wid);
		
		//Perform HSLThreshold to pull out only blue LED reflection of targets into BinaryImage
		//BinaryImage* binImg = img->ThresholdHSL(202, 255, 55, 255, 55, 129);      //RED LED WORKS TERRRIBLY!!!!
		BinaryImage* binImg = img->ThresholdHSL(52, 255, 71, 188, 76, 219);      //RED LED WORKS TERRRIBLY!!!!
		//BinaryImage* binImg = img->ThresholdHSL(57, 255, 79, 255, 51, 255);  //BLUE LED
		//BinaryImage* binImg = img->ThresholdHSL(159, 255, 0, 255, 71, 255);  //RED LED
		//Perform Morphology on image to remove noise/unwanted particles.  Also fills in incomplete particles.
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_PCLOSE);
		
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_DILATE);
		//Perform particle analysis on BinaryImage, creates vector with all particles found
		vector<ParticleAnalysisReport>* particles = binImg->GetOrderedParticleAnalysisReports();
		
		//Print numbers of particles found to driver station
		//m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "# Parts:%d    ",particles->size());
		//m_dsLCD->UpdateLCD();
		if(particles->size() > 1 || particles->size() < 30)
		{
			
			sort(particles->begin(),particles->end(),particleSort);
			
			unsigned x;
			int largestWidth = 0;
			int largestWidthVal = 0;
			
			for (x = 0; ((x < 3) && (x < particles->size())); x++)
			{
				if ((*particles)[x].imageWidth > largestWidthVal)
				{
					largestWidth = x;
					largestWidthVal = (*particles)[x].imageWidth;
				}
			}
			
			int distance;
			int smallestDistance = 0;
			int smallestDistanceVal = 999;
			
			for (x = 0; ((x < 3) && (x < particles->size())); x++)
			{
				distance = abs((*particles)[x].center_mass_x - (*particles)[largestWidth].center_mass_x);
				if (distance < smallestDistanceVal)
				{
					smallestDistance = x;
					smallestDistanceVal = distance;
				}
			}
			
			//dash->PutString("test","hi");
			//dash->PutDouble("Area 1:",(*particles)[1].particleArea);
			//dash->PutDouble("Area 2:",(*particles)[2].particleArea);
			//dash->PutDouble("Area 3:",(*particles)[3].particleArea);
			//dash->PutDouble("Area 4:",(*particles)[4].particleArea);
							
			//Prints X center of largest particle found
			//m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "%d     %d",fourLargest[0],fourLargest[1]);
			//m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "%f",(float)((*particles)[highestY].center_mass_y));
			//m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "High Y: %d",highestY);
			m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Cen X Norm:%f",(float)((*particles)[smallestDistance].center_mass_x_normalized));
			//m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "%d %d %d",(*particles)[0].center_mass_y,(*particles)[1].center_mass_y,(*particles)[2].center_mass_y);
			//m_dsLCD->UpdateLCD();
			
			//Returns the center of mass of the largest particle found (double)
			return (*particles)[smallestDistance].center_mass_x_normalized;	
		}
		else{return(0);}
	}
	
	state_t CameraHandler::getHotGoal () 
	{
		unsigned x;
		
		int largestWidth;		// Index of Particle
		int largestHeight;		// Index of Particle
		int largestWidthVal;	// Actual Width
		int largestHeightVal;	// Actual Height
		
		BinaryImage* binImg;
		vector<ParticleAnalysisReport>* particles;
		
		// Get Camera Image
		camera->GetImage(img);
		
		// Filter out Background
		binImg = img->ThresholdHSL(52, 255, 71, 188, 76, 219);
		
		// Make picture clear
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_PCLOSE);
		frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_DILATE);
		
		// Get Particle Analysis
		particles = binImg->GetOrderedParticleAnalysisReports();
		
		if (particles->size() == 1) {
			// Find Only One Particle
			return kNone;
		} else if (particles->size() > 0 && particles->size() < 30) {
			// Sort by size
			sort(particles->begin(), particles->end(), particleSort);
			
			// Initialize
			largestWidth = 0;
			largestHeight = 0;
			largestWidthVal = 0;
			largestHeightVal = 0;
			
			for (x=0; (x < 3 &&  x < particles->size()); x++) {
				// Find tallest
				if ((*particles)[x].boundingRect.height > largestHeightVal) {
					largestHeight = x;
					largestHeightVal = (*particles)[x].boundingRect.height;
				}
				
				// Find Fattest
				if ((*particles)[x].boundingRect.width > largestWidthVal) {
					largestWidth = x;
					largestWidthVal = (*particles)[x].boundingRect.width;
				}
			}

			if ((*particles)[largestWidth].center_mass_x < (*particles)[largestHeight].center_mass_x) {
				return kLeft;
			}
			else if ((*particles)[largestWidth].center_mass_x > (*particles)[largestHeight].center_mass_x) {
				return kRight;
			}
			else {
				return kNone;
			}
			
		} else {
			// Find Too Many Particles or None
			return kError;
		}
	}
	
	bool CameraHandler::getLeftHot ()
	{
		return getHotGoal() == kLeft;
	}
	
	bool CameraHandler::getRightHot ()
	{
		return getHotGoal() == kRight;
	}

