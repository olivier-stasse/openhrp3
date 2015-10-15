#include <iostream>
#include <string>
#include "omniORB4/CORBA.h"
#include "omniORB4/Naming.hh"
#include "omniorb-CORBA/Localization.hh"

using namespace std;

int main(int argc, char** argv)
{
  
  // Declare ORB
  CORBA::ORB_var orb;
  
  try {
    
    // Initialize the ORB
    orb = CORBA::ORB_init(argc, argv);
    
    // Get a reference to the Naming Service
    CORBA::Object_var rootContextObj = 
      orb->resolve_initial_references("NameService");
    CosNaming::NamingContext_var nc =
      CosNaming::NamingContext::_narrow(rootContextObj.in());
    
    CosNaming::Name name;
    name.length(1);
    name[0].id = (const char *) "Localization";
    name[0].kind = (const char *) "";
    // Invoke the root context to retrieve the object reference
    CORBA::Object_var managerObj = nc->resolve(name);

    // Narrow the previous object to obtain the correct type
    OpenHRP::Localization_var manager =
	OpenHRP::Localization::_narrow(managerObj.in());
    
    string info_in,exit,dummy;
    CORBA::String_var info_out;
    unsigned long key,shift;
    
    unsigned long int it=0;
    while(1)
      {
	// Ask for localization
	CORBA::String_var LinkName = (const char *) "WAIST";
	OpenHRP::LinkPosition aLinkPosition;
	manager->getLocalizationForLink(LinkName,aLinkPosition);
	
	// Display aLinkPosition.
	std::cout << "R: (";
	for(unsigned int i=0;i<3;i++)
	  {
	    for(unsigned int j=0;j<3;j++)
	      std::cout << aLinkPosition.R[i*3+j] << " ";
	    std::cout << std::endl;
	  }

	std::cout << "p(" ;
	for(unsigned int i=0;i<3;i++)
	  std::cout << aLinkPosition.p[i] << " ";
	std::cout << ")" << std::endl;
	// Wait
	usleep(1000000);

	it++;
	if (it>1000)
	  break;
      }

  }
  catch(const CORBA::Exception& e) {
    // Handles CORBA exceptions
    std::cerr << "Exception" << endl;
  }
}

