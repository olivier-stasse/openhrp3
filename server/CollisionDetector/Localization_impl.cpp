#include "CollisionDetector_impl.h"
#include "Localization_impl.h"

Localization_impl::Localization_impl(CORBA_ORB_ptr orb):
  CollisionDetector_impl_(0)
{
  
}

Localization_impl::~Localization_impl()
{
  
}

void Localization_impl::getLocalizationForLink(const char *aLinkName,
					       ::OpenHRP::LinkPosition & aLinkPosition,
					       ::CORBA::Double& simTime)
{
  if (CollisionDetector_impl_)
    {
      if (!CollisionDetector_impl_->getLocalizationForLink(aLinkName,
							   aLinkPosition,
							   simTime))
	{
	  for(unsigned int i=0;i<3;i++)
	    aLinkPosition.p[i]=0.0;
	  for(unsigned int i=0;i<9;i++)
	    aLinkPosition.R[i]=0.0;
	}
    }
}
void Localization_impl::setCollisionDetectorPtr(CollisionDetector_impl *aCDI)
{
  CollisionDetector_impl_ = aCDI;
}
