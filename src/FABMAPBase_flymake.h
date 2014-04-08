//=================================================================
//
//  FABMAPBase.h
//
//  Created by Mark Cummins and Paul Newman.
//  Copyright 2008 University of Oxford. All rights reserved.
//
//=================================================================
#pragma once

#include <MOOSUtilityLib/TMaxPair.h>
#include <vector>
#include <string>
#include <set>
#include <map>
#include <cassert>
#include <cmath>
#include <sstream>
#include <iostream>
#include <algorithm>

//****************************************************************************//
//   THIS CLASS DEALS WITH COMMS DETAILS. PUBLIC API IS DEFINED IN FABMAP.h   //
//****************************************************************************//

class Place
    {
    public:
        Place() 
        {
            dfProbWrtLastObservation = NAN;
            PlaceID = -1;
        }
        
        Place( const int &placeID )
        {
            dfProbWrtLastObservation = NAN;
            PlaceID = placeID;      
        }
        
        Place( const std::vector<std::string> &fileNames,
              const double &probWrtLastObs,
              const int &placeID ) 
        {
            FileNames = fileNames;
            dfProbWrtLastObservation = probWrtLastObs;
            PlaceID = placeID;
        }
        
        bool operator<( const Place &other ) const {
            return ( this->PlaceID < other.PlaceID );
        }
        
        std::vector<std::string> FileNames;
        double dfProbWrtLastObservation;
        int PlaceID;
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
                        unsigned int transactionID;
                        if(!MOOSValFromString(transactionID,Msg.GetString(),"TransactionID",true))
                        {
                            MOOSTrace("Badly formatted response from Fab_Map\n");
                            return -1;
                        }
                        if( transactionID == m_nTransactionID )
                        {
                            //std::cout << " Sent transaction " << m_nTransactionID << std::endl;
                            //std::cout << "  The reply I accepted was" << std::endl << "    "  << Msg.GetString() << std::endl << std::endl;
                            if ( !ParseFABMAPResult( Msg.GetString(), Images ) )
                            {
                                MOOSTrace("Failed to parse message: %s received from FabMap\n", Msg.GetString().c_str());
                                return -1;
                            }
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
        
        bool RecordPlaceDetails( const std::vector<std::string> &Images,
                                const int fabmapID )
        {
            // create bi-directional mapping for image filenames <--> FabMap ID
            FilenameToFabMapIDMapInsertReturn_t insertReturn;
            for(unsigned int i=0;i<Images.size();i++)
            {
                // Record the Filename --> FabMap ID association
                insertReturn = m_FilenameToFabMapIDMap.insert( FilenameToFabMapIDMap_t::value_type( Images[i], fabmapID ) );
                if ( !insertReturn.second )
                {
                    MOOSTrace( "You tried to add an image with the same name as a previously added image: %s", Images[i].c_str() );
                    return false;
                }
                
                // Record the FabMap ID --> Filename association
                m_FabMapIDToFilenameMap.insert( FabMapIDToFilenameMap_t::value_type( fabmapID, Images[i] ) );	    
            }
            return true;
        }
        
        bool ParseFABMAPResult( std::string sStr, 
                               const std::vector<std::string> &images )
        {
            // clear previous result
            m_Places.clear();
            
            // Sample Msg: TransactionID = 10,PDF=1:0.6,12:0.4,...,PlaceID=45  (place IDs start from 0)
            
            int numScenes;
            if ( !MOOSValFromString( numScenes, sStr, "SceneID", true ) )
            {
                MOOSTrace("Failed to find SceneID in string: %s\n", sStr.c_str() );
                return false;
            }
            
            int placeID; //= m_nTransactionID;
            if (!MOOSValFromString( placeID, sStr, "PlaceID", true ) )
            {
                MOOSTrace("Failed to find PlaceID in string: %s\n", sStr.c_str() );
                return false;
            }
            
            int numPlaces;
            if ( !MOOSValFromString( numPlaces, sStr, "NumPlaces", true ) )
            {
                MOOSTrace("Failed to find NumPlaces in string: %s\n", sStr.c_str() );
                return false;
            }
            
            if ( !RecordPlaceDetails( images, placeID ) )
            {
                MOOSTrace("Failed to record place details for place: %d\n", placeID );
                return false;
            }
            
            // can't use MOOSValFromString for PDF as it is not designed to handle comma
            // separated lists of values between '=' tokens and will just return the first
            // i.e. the result is that PDF=0:0.1,1:0.2,4:0.4 --> 0:0.1
            
            // strip transaction ID
            MOOSChomp( sStr, "," );
            // strip SceneID
            MOOSChomp( sStr, "," );
            // strip place ID
            MOOSChomp( sStr, "," );
            // strip number of places
            MOOSChomp( sStr, "," );   
            // strip header for PDF
            MOOSChomp( sStr, "=" );
            
            //std::cout << "PDF: " << sStr << std::endl;
            
            while(!sStr.empty())
            {
                std::string sPair = MOOSChomp(sStr);
                std::string sPlaceID = MOOSChomp(sPair,":");
                std::string sProb = sPair;
                unsigned int nID = atoi(sPlaceID.c_str());
                double dfProb = atof(sProb.c_str());
                
                // get the image filenames for the FabMap ID if we have them
                std::vector< std::string > cFilenames;    
                FabMapIDToFilenameMapConstItr_t startItr = m_FabMapIDToFilenameMap.lower_bound( nID );
                FabMapIDToFilenameMapConstItr_t endItr = m_FabMapIDToFilenameMap.upper_bound( nID );
                for ( FabMapIDToFilenameMapConstItr_t itr = startItr; 
                     itr != endItr; ++itr )
                {
                    cFilenames.push_back( itr->second );
                }
                
                m_Places.insert( Place( cFilenames, dfProb, nID ) );	    
            }
            
            return true;
        }
        
        double GetLocationProbability(std::string sFileName)
        {
            FilenameToFabMapIDMapConstItr_t itr = m_FilenameToFabMapIDMap.find( sFileName );
            if ( itr == m_FilenameToFabMapIDMap.end() )
            {
                MOOSTrace("no such place \"%s\"\n",sFileName.c_str());
                return 0.0;
            }
            
            return GetLocationProbability( itr->second );
        }
        
        double GetLocationProbability( int nPlace )
        {
            if( nPlace < 0 )
            {
                MOOSTrace("no such place %d\n",nPlace);
                return 0.0;	  
            }
            
            PlaceSetConstItr_t itr = m_Places.find( nPlace );
            if ( itr != m_Places.end() )
            {
                return itr->dfProbWrtLastObservation;
            }
            else
            {
                return 0.0;
            }
        }
        
        //find the place number, name , imagefile and probability of the most probable place with respect to the last image supplied
        bool GetMostProbablePlace( unsigned int & nPlace,std::string & sFileNames,double &dfProb)
        {
            TMaxPair<double,int> Max;
            for ( PlaceSetConstItr_t itr = m_Places.begin(); itr != m_Places.end(); ++itr )
            {
                Max.Update( itr->dfProbWrtLastObservation, itr->PlaceID );
            }
            
            if(!Max.Valid())
                return false;
            
            nPlace = (unsigned int)Max.Data();	
            
            PlaceSetConstItr_t itr = m_Places.find( Place( nPlace ) );
            
            dfProb = itr->dfProbWrtLastObservation;
            assert( dfProb == Max.Key() );
            
            if ( !itr->FileNames.empty() )
            {
                sFileNames = itr->FileNames[0];
            }
            
            //The final entry in m_Places represents the probability that
            //the last observation came from a new place not currently in the map.
            //if(nPlace == m_Places.size()-1)
            //    sFileNames = "NEW PLACE";
            
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
        
        typedef std::set< Place > PlaceSet_t;
        typedef PlaceSet_t::iterator PlaceSetItr_t;
        typedef PlaceSet_t::const_iterator PlaceSetConstItr_t;
        typedef std::pair< PlaceSetItr_t, bool > PlaceSetInsertReturn_t;
        
        typedef std::map< std::string, int > FilenameToFabMapIDMap_t;
        typedef FilenameToFabMapIDMap_t::iterator FilenameToFabMapIDMapItr_t;
        typedef FilenameToFabMapIDMap_t::const_iterator FilenameToFabMapIDMapConstItr_t;
        typedef std::pair< FilenameToFabMapIDMapItr_t, bool > FilenameToFabMapIDMapInsertReturn_t;
        
        typedef std::multimap< int, std::string > FabMapIDToFilenameMap_t;
        typedef FabMapIDToFilenameMap_t::iterator FabMapIDToFilenameMapItr_t;
        typedef FabMapIDToFilenameMap_t::const_iterator FabMapIDToFilenameMapConstItr_t;
        
        PlaceSet_t m_Places;
        
        //std::map<std::string , int> m_FilenameToFabMapIDMap;
        
        FilenameToFabMapIDMap_t m_FilenameToFabMapIDMap;
        FabMapIDToFilenameMap_t m_FabMapIDToFilenameMap;
        
        // note that Transaction ID does *NOT* necessarily equal PlaceID of last place added
        unsigned int m_nTransactionID;
    };
