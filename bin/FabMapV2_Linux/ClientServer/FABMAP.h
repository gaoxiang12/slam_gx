//=================================================================
//  FABMAP.h
//
//  Created by Mark Cummins and Paul Newman.
//  Copyright 2008 University of Oxford. All rights reserved.
//
//=================================================================

//**********************************************//
//   THIS CLASS DEFINES THE PUBLIC FabMap API   //
//**********************************************//

#define MOOSDBPORT 9000

#include "MOOSGenLib/MOOSGenLibGlobalHelper.h"
#include "MOOSLIB/MOOSCommClient.h"

#include "FABMAPBase.h"


class CFABMAPClient : public CFABMAPBase
    {
    private:
        typedef CFABMAPBase BASE;
        
    public:
        
        //Open the FABMAP Object and have it connect to the MOOSDB server (which handles inter-process comms)
        bool Open(const std::string & sMOOSDBHost="localhost", int nPort=MOOSDBPORT)
        {
            return BASE::Open(sMOOSDBHost,nPort);
        }
        
        //Post an image to the FABMAP Engine and wait for it to be processed.
        //
        //sImage is the path to an image.
        //
        //The FABMAP engine computes how likely each of the places in the map 
        //was to have generated the given image.
        //The image is also added to the map so that it can be recognised in future.
        //
        //The return value is the ID of the "place" added. This value can be used
        //in the GetLocationProbabiity method. 
        int ProcessImage(const std::string & sImage)
        {
            return BASE::ProcessImage(sImage);
        }
        
        //Post a set of images to the FABMAP Engine and wait for it to be processed.
        //
        //Here Images is a vector where each entry is a string specifying an image path
        //This call causes a bag-of-words to be computed from all these images and supplied to FabMap as a single observation.
        //Use this method if, for example, the robot has multiple cameras or collects panoramas using a pan-tilt.
        int ProcessImage(const std::vector<std::string> &Images)
        {
            return BASE::ProcessImage(Images);
        }
        
        //Find the place number, imagefile and probability of the most probable place
        bool GetMostProbablePlace( unsigned int & nPlace,std::string & sFileName,double &dfProb)
        {
            return BASE::GetMostProbablePlace(nPlace,sFileName,dfProb);
        }
        
        //Return the probability that the last image added is of the place nPlace
        double GetLocationProbabilty(int nPlace)
        {
            return BASE::GetLocationProbability(nPlace);
        }
        
        //Return the probability that the last image added is of the place previously created from sFileName
        double GetLocationProbabilty(const std::string &sFileName)
        {
            return BASE::GetLocationProbability(sFileName);
        }
        
        //Return the pdf over places due to the last image
        //- that is, for each place, the probability that the image came from that place
        //The final entry in the returned vector is special. 
        //It corresponds to to the probability that the image came from none of the places in the map
        //but rather from a new place.
        //Note that due to a limitation on MOOS message lengths for inter-process comms, the pdf filled in for only the 100 most likely places. Other probabilities are set to 0. 
        std::set<Place> GetPDF()
        {
            return m_Places;
        }
        
    };
