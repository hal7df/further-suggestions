#include "camera.h"
        
        cameraHandler::cameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, SmartDashboard *dash)       
        {
                //This runs once when cameraHandler Class is initilized
                camera->WriteResolution(AxisCamera::kResolution_320x240);
                camera->WriteBrightness(40);

                this->img = new ColorImage(IMAQ_IMAGE_HSL);
                this->camera = camera;
                this->m_dsLCD = m_dsLCD;
                this->dash = dash;
        }
        
        bool particleSort (ParticleAnalysisReport i, ParticleAnalysisReport j) {return (i.particleArea > j.particleArea);}
        
        
        double cameraHandler::getCenter()
        {
                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Test");
                m_dsLCD->UpdateLCD();
                //Get new camera image
                camera->GetImage(img);
                
                //img.Write("bob2.jpg");  //Cannot work with non-RGB images
                
                //int Wid = img->GetWidth();  //Sanity Check: Check width of image
                //Prints width of image from camera
                //m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1,"Width of Image: %d",Wid);
                
                //Perform HSLThreshold to pull out only blue LED reflection of targets into BinaryImage
                BinaryImage* binImg = img->ThresholdHSL(83, 245, 0, 255, 122, 255);
                
                //Perform Morphology on image to remove noise/unwanted particles.  Also fills in incomplete particles.
                frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_POPEN);
                
                frcMorphology(binImg->GetImaqImage(),binImg->GetImaqImage(),IMAQ_DILATE);
                //Perform particle analysis on BinaryImage, creates vector with all particles found
                vector<ParticleAnalysisReport>* particles = binImg->GetOrderedParticleAnalysisReports();
                
                //Print numbers of particles found to driver station
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "# Parts:%d    ",particles->size());
                m_dsLCD->UpdateLCD();
                if(particles->size() > 1 || particles->size() < 15 )
                {
                        
                        sort(particles->begin(),particles->end(),particleSort);
                        
                        
                        int highestY = 0;
                        int highestYVal = 999;
                        for(unsigned int x = 0; ((x < 4) && (x < particles->size()));x++)
                        {
                                if((*particles)[x].center_mass_y < highestYVal)
                                {
                                        highestYVal = (*particles)[x].center_mass_y;
                                        highestY = x;
                                }
                        }
                        
                        //dash->PutString("test","hi");
                        //dash->PutDouble("Area 1:",(*particles)[1].particleArea);
                        //dash->PutDouble("Area 2:",(*particles)[2].particleArea);
                        //dash->PutDouble("Area 3:",(*particles)[3].particleArea);
                        //dash->PutDouble("Area 4:",(*particles)[4].particleArea);
                                                        
                        //Prints X center of largest particle found
                        //m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "%d     %d",fourLargest[0],fourLargest[1]);
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "%f",(float)((*particles)[highestY].center_mass_y));
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "High Y: %d",highestY);
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Cen X Norm:%f",(float)((*particles)[highestY].center_mass_x_normalized));
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "%d %d %d",(*particles)[0].center_mass_y,(*particles)[1].center_mass_y,(*particles)[2].center_mass_y);
                        m_dsLCD->UpdateLCD();
                        
                        //Returns the center of mass of the largest particle found (double)
                        return (*particles)[highestY].center_mass_x_normalized; 
                }
                else{return(0);}
        }
