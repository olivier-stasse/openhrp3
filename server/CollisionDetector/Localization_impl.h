#ifndef OPENHRP_LOCALIZATION_IMPL_H_INCLUDED
#define OPENHRP_LOCALIZATION_IMPL_H_INCLUDED

#include <hrpCorba/ORBwrap.h>
#include "hrpCorba/Localization.hh"
using namespace std;
using namespace OpenHRP;


class CollisionDetector_impl;
class Localization_impl :virtual public POA_OpenHRP::Localization,
  virtual public PortableServer::RefCountServantBase
{
 public:
  Localization_impl(CORBA_ORB_ptr orb);
  ~Localization_impl();
  
  void getLocalizationForLink(const char *aLinkName,
			      ::OpenHRP::LinkPosition & aLinkPosition);

  void setCollisionDetectorPtr(CollisionDetector_impl *);

 private:
  CollisionDetector_impl* CollisionDetector_impl_;
};

#endif /* OPENHRP_LOCALIZATION_IMPL_H_INCLUDED */
