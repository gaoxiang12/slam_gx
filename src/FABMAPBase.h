//=================================================================
//
//  FABMAPBase.h
//
//  Created by Mark Cummins and Paul Newman.
//  Copyright 2008 University of Oxford. All rights reserved.
//
//=================================================================

#include <MOOSUtilityLib/TMaxPair.h>
#include <vector>
#include <string>
#include <map>
#include <cassert>
#include <sstream>
#include <iostream>
#include <algorithm>

    //****************************************************************************//
    //   THIS CLASS DEALS WITH COMMS DETAILS. PUBLIC API IS DEFINED IN FABMAP.h   //
    //****************************************************************************//

struct Place
{
    std::vector<std::string> FileNames;
    double dfProbWrtLastObservation;
};

class CFABMAPBase
{
    static bool _CCB(void * pParam)
    {
        CFABMAPBase* pMe = (CFABMAPBase*) pParam;
        return pMe->OnMOOSConnect();
    }
    static bool _DCCB(void * pParam)
    {
        CFABMAPBase* pMe = (CFABMAPBase*) pParam;
        return pMe->OnMOOSDisconnect();
    }
    
protected:
    CMOOSCommClient m_Comms;
    std::string m_sHost;
    
    bool Open(const std::string & sMOOSDBHost, int nPort=MOOSDBPORT)
    {
        m_nTransactionID=0;
        m_Comms.SetOnConnectCallBack(_CCB, this);
        m_Comms.SetOnDisconnectCallBack(_DCCB, this);
        
        //quiet comms - don't want to confuse user
        m_Comms.SetQuiet(true);
        bool bSuccess = m_Comms.Run(sMOOSDBHost.c_str(),nPort,"FABMAPClient");
        MOOSPause(1000);
        m_Comms.Notify("FABMAP_RESET","1");
        MOOSPause(1000);
        
        if(m_Comms.IsConnected())
        {
            return bSuccess;
        }
        else
        {
            MOOSTrace("\n ERROR: Failed to connect to MOOS DB. Is it running?\n\n");
            return false;
        }
    }
    
    int ProcessImage(const std::string &sImage)
    {
        return ProcessImage(std::vector<std::string>(1,sImage));
    }

    int ProcessImage(const std::vector<std::string> &Images)
    { 
        if(!m_Comms.IsConnected())
        {
            std::cout<< "No connection to MOOS DB - is it running?" << std::endl;
            return -1;
        }

        std::stringstream ss;
        ss<<"TransactionID="<<m_nTransactionID<<",";
        ss<<"ImagePaths=";
        for(unsigned int i=0;i<Images.size();i++)
        {
            ss<<Images[i]<<";";
        }
        
        std::cout<<"Submitting image for processing..."<<std::endl;
        m_Comms.Notify("FABMAP_INPUT",ss.str());
        
        //unusually we are going to make this a blocking call
        double dfStart = MOOSTime();
        while(MOOSTime()-dfStart<20.0)
        {
            MOOSMSG_LIST Mail;
            if(m_Comms.Fetch(Mail))
            {
                CMOOSMsg Msg;
                if(m_Comms.PeekMail(Mail,"FABMAP_OUTPUT",Msg,true))
                {
                    unsigned int nID;
                    if(!MOOSValFromString(nID,Msg.GetString(),"TransactionID",true))
                    {
                        MOOSTrace("Badly formatted response from Fab_Map\n");
                        return -1;
                    }
                    if(nID==m_nTransactionID)
                    {
                        //std::cout << " Sent transaction " << m_nTransactionID << std::endl;
                        //std::cout << "  The reply I accepted was" << std::endl << "    "  << Msg.GetString() << std::endl << std::endl;
                        RecordPlaceDetails(Images);
                        ParseFABMAPResult(Msg.GetString());            
                        return ++m_nTransactionID-1;
                    }
                    else
                    {
                        //std::cout << "Discarding message. Bad transaction ID. Possibly from previous run." << std::endl << "  (ID was " << nID << ", expected " << m_nTransactionID << ")" << std::endl;
                    }
                }
            }

            //nothing yet - wait...
            MOOSPause(100);
        }
        
        MOOSTrace("   no reply after 20 seconds...giving up\n");
        MOOSTrace("   Are WordMaker and FabMap running?\n");
        return -1;
    }
   
    void RecordPlaceDetails(const std::vector<std::string> &Images)
    {
        Place thisPlace;
        thisPlace.FileNames = Images;
        m_Places.push_back(thisPlace);
        assert(m_Places.size() == m_nTransactionID+1);
        for(unsigned int i=0;i<Images.size();i++)
        {
            m_FilenameToIndexMap.insert(std::make_pair(Images[i],m_nTransactionID)); //Record the association
        }
    }

    bool ParseFABMAPResult(std::string  sStr)
    {
        //clear previous result
        for(unsigned int i = 0;i<m_Places.size();i++)
            m_Places[i].dfProbWrtLastObservation = 0.0;
        
        //TransactionID = 10,PDF=1:0.6,12:0.4  (place IDs start from 0)
        MOOSChomp(sStr,",");MOOSChomp(sStr,"="); //Strip message header
        while(!sStr.empty())
        {
            std::string sPair = MOOSChomp(sStr);
            std::string sPlaceID = MOOSChomp(sPair,":");
            std::string sProb = sPair;
            unsigned int nID = atoi(sPlaceID.c_str());
            double dfProb = atof(sProb.c_str());
            if(nID>=0 && nID<m_Places.size())
            {
                m_Places[nID].dfProbWrtLastObservation = dfProb;
            }
            else
            {
                MOOSTrace("no such place: %d\n");
                return false;
            }
        }
        return true;
    }
          
    double GetLocationProbability(std::string sFileName)
    {
        std::map<std::string, unsigned int>::iterator q = m_FilenameToIndexMap.find(sFileName);
       
        if(q==m_FilenameToIndexMap.end())
        {
            MOOSTrace("no such place \"%s\"\n",sFileName.c_str());
            return 0.0;
        }
        
        return   GetLocationProbability(q->second);
    }
    
    double GetLocationProbability(unsigned int  nPlace)
    {
        if(nPlace>=0 && nPlace<m_Places.size())
            return m_Places[nPlace].dfProbWrtLastObservation;
        else
        {
            MOOSTrace("no such place %d\n",nPlace);
            return 0.0;
        }
    }
    
    //find the place number, name , imagefile and probability of the most probable place with respect to the last image supplied
    bool GetMostProbablePlace( unsigned int & nPlace,std::string & sFileNames,double &dfProb)
    {
        TMaxPair<double,int> Max;
        for(unsigned int k = 0;k<m_Places.size();k++)
            Max.Update(m_Places[k].dfProbWrtLastObservation,k);
        
        if(!Max.Valid())
            return false;
        
        unsigned int i = Max.Data();
        nPlace = i;
        sFileNames = m_Places[i].FileNames[0];
        dfProb = m_Places[i].dfProbWrtLastObservation;
        
        //The final entry in m_Places represents the probability that
        //the last observation came from a new place not currently in the map.
        if(nPlace == m_Places.size()-1)
            sFileNames = "NEW PLACE";

        assert(dfProb==Max.Key());
        return true;
    }
        
    bool OnMOOSDisconnect()
    {
        return true;
    }
    
    bool OnMOOSConnect()
    {
        m_Comms.Register("FABMAP_OUTPUT",0);
        return true;
    }

protected:

    std::vector<Place> m_Places;
    std::map<std::string , unsigned int> m_FilenameToIndexMap;
    unsigned int m_nTransactionID;
};
