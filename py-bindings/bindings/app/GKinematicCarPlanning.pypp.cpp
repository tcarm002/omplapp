// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/app.h"
#include "GKinematicCarPlanning.pypp.hpp"

namespace bp = boost::python;

struct GKinematicCarPlanning_wrapper : ompl::app::GKinematicCarPlanning, bp::wrapper< ompl::app::GKinematicCarPlanning > {

    GKinematicCarPlanning_wrapper( )
    : ompl::app::GKinematicCarPlanning( )
      , bp::wrapper< ompl::app::GKinematicCarPlanning >(){
        // null constructor
    
    }

    virtual void clear(  ) {
        if( bp::override func_clear = this->get_override( "clear" ) )
            func_clear(  );
        else{
            this->ompl::control::SimpleSetup::clear(  );
        }
    }
    
    void default_clear(  ) {
        ompl::control::SimpleSetup::clear( );
    }

    void computeGeometrySpecification(  ){
        ompl::app::RigidBodyGeometry::computeGeometrySpecification(  );
    }

    static ::ompl::control::ControlSpacePtr constructControlSpace(  ){
        return ompl::app::KinematicCarPlanning::constructControlSpace(  );
    }

    static ::ompl::base::StateSpacePtr constructStateSpace(  ){
        return ompl::app::KinematicCarPlanning::constructStateSpace(  );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getDefaultStartState(  ) const  {
        if( bp::override func_getDefaultStartState = this->get_override( "getDefaultStartState" ) )
            return func_getDefaultStartState(  );
        else{
            return this->ompl::app::KinematicCarPlanning::getDefaultStartState(  );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getDefaultStartState(  ) const  {
        return ompl::app::KinematicCarPlanning::getDefaultStartState( );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getFullStateFromGeometricComponent( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state ) const  {
        if( bp::override func_getFullStateFromGeometricComponent = this->get_override( "getFullStateFromGeometricComponent" ) )
            return func_getFullStateFromGeometricComponent( boost::ref(state) );
        else{
            return this->ompl::app::KinematicCarPlanning::getFullStateFromGeometricComponent( boost::ref(state) );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getFullStateFromGeometricComponent( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state ) const  {
        return ompl::app::KinematicCarPlanning::getFullStateFromGeometricComponent( boost::ref(state) );
    }

    virtual ::ompl::base::ScopedState< ompl::base::StateSpace > getGeometricComponentState( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state, unsigned int index ) const  {
        if( bp::override func_getGeometricComponentState = this->get_override( "getGeometricComponentState" ) )
            return func_getGeometricComponentState( boost::ref(state), index );
        else{
            return this->ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState( boost::ref(state), index );
        }
    }
    
    ::ompl::base::ScopedState< ompl::base::StateSpace > default_getGeometricComponentState( ::ompl::base::ScopedState< ompl::base::StateSpace > const & state, unsigned int index ) const  {
        return ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState( boost::ref(state), index );
    }

    virtual ::ompl::base::State const * getGeometricComponentStateInternal( ::ompl::base::State const * state, unsigned int arg1 ) const {
        if( bp::override func_getGeometricComponentStateInternal = this->get_override( "getGeometricComponentStateInternal" ) )
            return func_getGeometricComponentStateInternal( boost::python::ptr(state), arg1 );
        else{
            return this->ompl::app::KinematicCarPlanning::getGeometricComponentStateInternal( boost::python::ptr(state), arg1 );
        }
    }
    
    virtual ::ompl::base::State const * default_getGeometricComponentStateInternal( ::ompl::base::State const * state, unsigned int arg1 ) const {
        return ompl::app::KinematicCarPlanning::getGeometricComponentStateInternal( boost::python::ptr(state), arg1 );
    }

    virtual unsigned int getRobotCount(  ) const  {
        if( bp::override func_getRobotCount = this->get_override( "getRobotCount" ) )
            return func_getRobotCount(  );
        else{
            return this->ompl::app::KinematicCarPlanning::getRobotCount(  );
        }
    }
    
    unsigned int default_getRobotCount(  ) const  {
        return ompl::app::KinematicCarPlanning::getRobotCount( );
    }

    virtual void inferEnvironmentBounds(  ) {
        if( bp::override func_inferEnvironmentBounds = this->get_override( "inferEnvironmentBounds" ) )
            func_inferEnvironmentBounds(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds(  );
        }
    }
    
    void default_inferEnvironmentBounds(  ) {
        ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds( );
    }

    virtual void inferProblemDefinitionBounds(  ) {
        if( bp::override func_inferProblemDefinitionBounds = this->get_override( "inferProblemDefinitionBounds" ) )
            func_inferProblemDefinitionBounds(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds(  );
        }
    }
    
    void default_inferProblemDefinitionBounds(  ) {
        ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds( );
    }

    virtual bool isSelfCollisionEnabled(  ) const  {
        if( bp::override func_isSelfCollisionEnabled = this->get_override( "isSelfCollisionEnabled" ) )
            return func_isSelfCollisionEnabled(  );
        else{
            return this->ompl::app::KinematicCarPlanning::isSelfCollisionEnabled(  );
        }
    }
    
    bool default_isSelfCollisionEnabled(  ) const  {
        return ompl::app::KinematicCarPlanning::isSelfCollisionEnabled( );
    }

    virtual void ode( ::std::vector< double > const & q, ::ompl::control::Control const * ctrl, ::std::vector< double > & qdot ){
        if( bp::override func_ode = this->get_override( "ode" ) )
            func_ode( boost::ref(q), boost::python::ptr(ctrl), boost::ref(qdot) );
        else{
            this->ompl::app::KinematicCarPlanning::ode( boost::ref(q), boost::python::ptr(ctrl), boost::ref(qdot) );
        }
    }
    
    virtual void default_ode( ::std::vector< double > const & q, ::ompl::control::Control const * ctrl, ::std::vector< double > & qdot ){
        ompl::app::KinematicCarPlanning::ode( boost::ref(q), boost::python::ptr(ctrl), boost::ref(qdot) );
    }

    virtual void postPropagate( ::ompl::base::State const * state, ::ompl::control::Control const * control, double const duration, ::ompl::base::State * result ){
        if( bp::override func_postPropagate = this->get_override( "postPropagate" ) )
            func_postPropagate( boost::python::ptr(state), boost::python::ptr(control), duration, boost::python::ptr(result) );
        else{
            this->ompl::app::KinematicCarPlanning::postPropagate( boost::python::ptr(state), boost::python::ptr(control), duration, boost::python::ptr(result) );
        }
    }
    
    virtual void default_postPropagate( ::ompl::base::State const * state, ::ompl::control::Control const * control, double const duration, ::ompl::base::State * result ){
        ompl::app::KinematicCarPlanning::postPropagate( boost::python::ptr(state), boost::python::ptr(control), duration, boost::python::ptr(result) );
    }

    virtual void setDefaultControlBounds(  ) {
        if( bp::override func_setDefaultControlBounds = this->get_override( "setDefaultControlBounds" ) )
            func_setDefaultControlBounds(  );
        else{
            this->ompl::app::KinematicCarPlanning::setDefaultControlBounds(  );
        }
    }
    
    void default_setDefaultControlBounds(  ) {
        ompl::app::KinematicCarPlanning::setDefaultControlBounds( );
    }

    virtual void setup(  ) {
        if( bp::override func_setup = this->get_override( "setup" ) )
            func_setup(  );
        else{
            this->ompl::app::AppBase< (ompl::app::AppType)1 >::setup(  );
        }
    }
    
    void default_setup(  ) {
        ompl::app::AppBase< (ompl::app::AppType)1 >::setup( );
    }

    virtual ::ompl::base::PlannerStatus solve( double time=1.0e+0 ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( time );
        else{
            return this->ompl::control::SimpleSetup::solve( time );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( double time=1.0e+0 ) {
        return ompl::control::SimpleSetup::solve( time );
    }

    virtual ::ompl::base::PlannerStatus solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( boost::ref(ptc) );
        else{
            return this->ompl::control::SimpleSetup::solve( boost::ref(ptc) );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        return ompl::control::SimpleSetup::solve( boost::ref(ptc) );
    }

};

void register_GKinematicCarPlanning_class(){

    { //::ompl::app::GKinematicCarPlanning
        typedef bp::class_< GKinematicCarPlanning_wrapper, bp::bases< ompl::app::KinematicCarPlanning, ompl::app::RenderGeometry >, boost::noncopyable > GKinematicCarPlanning_exposer_t;
        GKinematicCarPlanning_exposer_t GKinematicCarPlanning_exposer = GKinematicCarPlanning_exposer_t( "GKinematicCarPlanning", bp::init< >() );
        bp::scope GKinematicCarPlanning_scope( GKinematicCarPlanning_exposer );
        { //::ompl::app::RigidBodyGeometry::computeGeometrySpecification
        
            typedef void ( GKinematicCarPlanning_wrapper::*computeGeometrySpecification_function_type)(  ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "computeGeometrySpecification"
                , computeGeometrySpecification_function_type( &GKinematicCarPlanning_wrapper::computeGeometrySpecification ) );
        
        }
        { //::ompl::app::KinematicCarPlanning::constructControlSpace
        
            typedef ::ompl::control::ControlSpacePtr ( *constructControlSpace_function_type )(  );
            
            GKinematicCarPlanning_exposer.def( 
                "constructControlSpace"
                , constructControlSpace_function_type( &GKinematicCarPlanning_wrapper::constructControlSpace ) );
        
        }
        { //::ompl::app::KinematicCarPlanning::constructStateSpace
        
            typedef ::ompl::base::StateSpacePtr ( *constructStateSpace_function_type )(  );
            
            GKinematicCarPlanning_exposer.def( 
                "constructStateSpace"
                , constructStateSpace_function_type( &GKinematicCarPlanning_wrapper::constructStateSpace ) );
        
        }
        { //::ompl::app::KinematicCarPlanning::getDefaultStartState
        
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( ::ompl::app::KinematicCarPlanning::*getDefaultStartState_function_type)(  ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( GKinematicCarPlanning_wrapper::*default_getDefaultStartState_function_type)(  ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "getDefaultStartState"
                , getDefaultStartState_function_type(&::ompl::app::KinematicCarPlanning::getDefaultStartState)
                , default_getDefaultStartState_function_type(&GKinematicCarPlanning_wrapper::default_getDefaultStartState) );
        
        }
        { //::ompl::app::KinematicCarPlanning::getFullStateFromGeometricComponent
        
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( ::ompl::app::KinematicCarPlanning::*getFullStateFromGeometricComponent_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const & ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( GKinematicCarPlanning_wrapper::*default_getFullStateFromGeometricComponent_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const & ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "getFullStateFromGeometricComponent"
                , getFullStateFromGeometricComponent_function_type(&::ompl::app::KinematicCarPlanning::getFullStateFromGeometricComponent)
                , default_getFullStateFromGeometricComponent_function_type(&GKinematicCarPlanning_wrapper::default_getFullStateFromGeometricComponent)
                , ( bp::arg("state") ) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState
        
            typedef ompl::app::GKinematicCarPlanning exported_class_t;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( exported_class_t::*getGeometricComponentState_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const &,unsigned int ) const;
            typedef ::ompl::base::ScopedState< ompl::base::StateSpace > ( GKinematicCarPlanning_wrapper::*default_getGeometricComponentState_function_type)( ::ompl::base::ScopedState< ompl::base::StateSpace > const &,unsigned int ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "getGeometricComponentState"
                , getGeometricComponentState_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::getGeometricComponentState)
                , default_getGeometricComponentState_function_type(&GKinematicCarPlanning_wrapper::default_getGeometricComponentState)
                , ( bp::arg("state"), bp::arg("index") ) );
        
        }
        { //::ompl::app::KinematicCarPlanning::getGeometricComponentStateInternal
        
            typedef ::ompl::base::State const * ( GKinematicCarPlanning_wrapper::*getGeometricComponentStateInternal_function_type)( ::ompl::base::State const *,unsigned int ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "getGeometricComponentStateInternal"
                , getGeometricComponentStateInternal_function_type( &GKinematicCarPlanning_wrapper::default_getGeometricComponentStateInternal )
                , ( bp::arg("state"), bp::arg("arg1") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::app::KinematicCarPlanning::getGeometricComponentStateSpace
        
            typedef ::ompl::base::StateSpacePtr const & ( ::ompl::app::KinematicCarPlanning::*getGeometricComponentStateSpace_function_type)(  ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "getGeometricComponentStateSpace"
                , getGeometricComponentStateSpace_function_type(&::ompl::app::KinematicCarPlanning::getGeometricComponentStateSpace)
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::app::KinematicCarPlanning::getRobotCount
        
            typedef unsigned int ( ::ompl::app::KinematicCarPlanning::*getRobotCount_function_type)(  ) const;
            typedef unsigned int ( GKinematicCarPlanning_wrapper::*default_getRobotCount_function_type)(  ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "getRobotCount"
                , getRobotCount_function_type(&::ompl::app::KinematicCarPlanning::getRobotCount)
                , default_getRobotCount_function_type(&GKinematicCarPlanning_wrapper::default_getRobotCount) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds
        
            typedef ompl::app::GKinematicCarPlanning exported_class_t;
            typedef void ( exported_class_t::*inferEnvironmentBounds_function_type)(  ) ;
            typedef void ( GKinematicCarPlanning_wrapper::*default_inferEnvironmentBounds_function_type)(  ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "inferEnvironmentBounds"
                , inferEnvironmentBounds_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::inferEnvironmentBounds)
                , default_inferEnvironmentBounds_function_type(&GKinematicCarPlanning_wrapper::default_inferEnvironmentBounds) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds
        
            typedef ompl::app::GKinematicCarPlanning exported_class_t;
            typedef void ( exported_class_t::*inferProblemDefinitionBounds_function_type)(  ) ;
            typedef void ( GKinematicCarPlanning_wrapper::*default_inferProblemDefinitionBounds_function_type)(  ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "inferProblemDefinitionBounds"
                , inferProblemDefinitionBounds_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::inferProblemDefinitionBounds)
                , default_inferProblemDefinitionBounds_function_type(&GKinematicCarPlanning_wrapper::default_inferProblemDefinitionBounds) );
        
        }
        { //::ompl::app::KinematicCarPlanning::isSelfCollisionEnabled
        
            typedef bool ( ::ompl::app::KinematicCarPlanning::*isSelfCollisionEnabled_function_type)(  ) const;
            typedef bool ( GKinematicCarPlanning_wrapper::*default_isSelfCollisionEnabled_function_type)(  ) const;
            
            GKinematicCarPlanning_exposer.def( 
                "isSelfCollisionEnabled"
                , isSelfCollisionEnabled_function_type(&::ompl::app::KinematicCarPlanning::isSelfCollisionEnabled)
                , default_isSelfCollisionEnabled_function_type(&GKinematicCarPlanning_wrapper::default_isSelfCollisionEnabled) );
        
        }
        { //::ompl::app::KinematicCarPlanning::ode
        
            typedef void ( GKinematicCarPlanning_wrapper::*ode_function_type)( ::std::vector< double > const &,::ompl::control::Control const *,::std::vector< double > & ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "ode"
                , ode_function_type( &GKinematicCarPlanning_wrapper::default_ode )
                , ( bp::arg("q"), bp::arg("ctrl"), bp::arg("qdot") ) );
        
        }
        { //::ompl::app::KinematicCarPlanning::postPropagate
        
            typedef void ( GKinematicCarPlanning_wrapper::*postPropagate_function_type)( ::ompl::base::State const *,::ompl::control::Control const *,double const,::ompl::base::State * ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "postPropagate"
                , postPropagate_function_type( &GKinematicCarPlanning_wrapper::default_postPropagate )
                , ( bp::arg("state"), bp::arg("control"), bp::arg("duration"), bp::arg("result") ) );
        
        }
        { //::ompl::app::KinematicCarPlanning::setDefaultControlBounds
        
            typedef void ( ::ompl::app::KinematicCarPlanning::*setDefaultControlBounds_function_type)(  ) ;
            typedef void ( GKinematicCarPlanning_wrapper::*default_setDefaultControlBounds_function_type)(  ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "setDefaultControlBounds"
                , setDefaultControlBounds_function_type(&::ompl::app::KinematicCarPlanning::setDefaultControlBounds)
                , default_setDefaultControlBounds_function_type(&GKinematicCarPlanning_wrapper::default_setDefaultControlBounds) );
        
        }
        { //::ompl::app::AppBase< (ompl::app::AppType)1 >::setup
        
            typedef ompl::app::GKinematicCarPlanning exported_class_t;
            typedef void ( exported_class_t::*setup_function_type)(  ) ;
            typedef void ( GKinematicCarPlanning_wrapper::*default_setup_function_type)(  ) ;
            
            GKinematicCarPlanning_exposer.def( 
                "setup"
                , setup_function_type(&::ompl::app::AppBase< (ompl::app::AppType)1 >::setup)
                , default_setup_function_type(&GKinematicCarPlanning_wrapper::default_setup) );
        
        }
        GKinematicCarPlanning_exposer.staticmethod( "constructControlSpace" );
        GKinematicCarPlanning_exposer.staticmethod( "constructStateSpace" );
        GKinematicCarPlanning_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::app::GKinematicCarPlanning::*)( double ))(&::ompl::app::GKinematicCarPlanning::solve), (bp::arg("solveTime")) );
        GKinematicCarPlanning_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::app::GKinematicCarPlanning::*)( const ompl::base::PlannerTerminationCondition& ))(&::ompl::app::GKinematicCarPlanning::solve), (bp::arg("ptc")) );
        GKinematicCarPlanning_exposer.def("clear", &GKinematicCarPlanning_wrapper::clear);
        GKinematicCarPlanning_exposer.def("getStateSpace", &::ompl::control::SimpleSetup::getStateSpace, bp::return_value_policy< bp::copy_const_reference >());
    }

}
