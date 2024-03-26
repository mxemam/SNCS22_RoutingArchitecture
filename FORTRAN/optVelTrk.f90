! gfortran optVelTrk.f90 ./libocpiddae1.a ./libfgssqp.a ./libspline.a ./libroadhelper.a
! ./a.out
MODULE RoadSpline
   IMPLICIT NONE
   INTEGER, SAVE :: nrRoadPoints
   DOUBLEPRECISION, DIMENSION(500, 2), SAVE :: roadDataMatrix

CONTAINS
   SUBROUTINE InitRoadData(dataFileName, nrDataPoints, fcnStatus)
      IMPLICIT NONE
      CHARACTER(*), INTENT(IN)    :: dataFileName
      INTEGER, INTENT(IN)         :: nrDataPoints
      INTEGER, INTENT(OUT)        :: fcnStatus
      ! Local variables
      INTEGER                     :: idx

      fcnStatus = 1
      ! read data
      nrRoadPoints = nrDataPoints
      OPEN (1, FILE=dataFileName, STATUS='OLD')
      DO idx = 1, nrRoadPoints
         READ (1, *, ERR=30) roadDataMatrix(idx, 1:2)
      END DO
      CLOSE (1)
      RETURN
30    CONTINUE
      fcnStatus = -1
      WRITE (*, *) "ERROR2 READING DATA FILE! ROAD COULD NOT BE INITIALIZED. RETURNING ..."
      RETURN
   END SUBROUTINE InitRoadData

   SUBROUTINE FindArcLengthIdx(arcLen, arcLenIdx)
      IMPLICIT NONE
      ! INOUT ARGUMENTS
      DOUBLEPRECISION, INTENT(IN) :: arcLen
      INTEGER, INTENT(OUT)        :: arcLenIdx
      ! Local variables
      INTEGER                     :: idx, ltIdx, rtIdx ! Indices to search the array
      DOUBLEPRECISION             :: arcLenTmp

      ! Ensure that the given arc length value lies within the road limits
      arcLenTmp = Max(roadDataMatrix(1, 1), Min(roadDataMatrix(nrRoadPoints, 1), arcLen))

      ! Initialize index pointers to check the array data
      ltIdx = 0
      rtIdx = nrRoadPoints
      ! Note that the algorithm finds the solution in O(log(n)) steps
      DO idx = 1, nrRoadPoints + 1
         IF (ltIdx > rtIdx) STOP ! Solution found in the previous step, exit
         ! Check the element in the middle of the left/right indices
         arcLenIdx = ltIdx + ((rtIdx - ltIdx)/2)
         IF ((arcLenTmp >= roadDataMatrix(arcLenIdx, 1)) .AND. (arcLenTmp <= roadDataMatrix(arcLenIdx + 1, 1))) THEN
            ! Point with the closest arc length was found
            RETURN
         ELSE
            ! Decide which half to check next (left or right)
            IF (arcLen < roadDataMatrix(arcLenIdx, 1)) THEN
               ! Check left half
               rtIdx = arcLenIdx
            ELSE
               ! Check right half
               ltIdx = arcLenIdx + 1
            END IF
         END IF
      END DO
      RETURN
   END SUBROUTINE FindArcLengthIdx

   SUBROUTINE GetRoadMaxSpeed(arcLen, maxSpeed)
      Use RoadHelper
      Use ROADMODULE
      IMPLICIT NONE
      ! INOUT ARGUMENTS
      DOUBLEPRECISION, INTENT(IN)     :: arcLen
      DOUBLEPRECISION, INTENT(OUT)    :: maxSpeed
      ! Local variables
      INTEGER                         :: idx

      ! Find the element with the closest arc length value
      CALL FindClosestArcLength(arcLen, idx)
      ! Convert to the max speed matrix index
      maxSpeed = arcLen * (roadDataMatrix(nrRoadPoints, 1) - roadDataMatrix(1, 1))/(ROAD_S(ROAD_NPOINTS) - ROAD_S(0))
      CALL FindArcLengthIdx(maxSpeed, idx)

      IF (idx < nrRoadPoints) THEN
         IF (roadDataMatrix(idx + 1, 2) == roadDataMatrix(idx, 2)) THEN
            maxSpeed = roadDataMatrix(idx, 2)
         ELSE
            ! Interpolate to find speed: v = v_2 - ((v_2 - v_1)*(s_2 - s)/(s_2 - s_1))
            maxSpeed = (roadDataMatrix(idx + 1, 1) - maxSpeed)/(roadDataMatrix(idx + 1, 1) - roadDataMatrix(idx, 1))
            maxSpeed = roadDataMatrix(idx + 1, 2) - &
               (roadDataMatrix(idx + 1, 2) - roadDataMatrix(idx, 2))*maxSpeed
         END IF
      ELSE
         maxSpeed = roadDataMatrix(nrRoadPoints, 2)
      END IF
      RETURN
   END SUBROUTINE GetRoadMaxSpeed
END MODULE RoadSpline

PROGRAM optVelTrk
   USE ROADMODULE
   USE RoadHelper
   USE RoadSpline
   IMPLICIT NONE
   !------------------------------------------------------------------------------------------------------------
   ! Don't change the following declarations
   INTEGER                                         :: IRES, IREALTIME, NREALTIME, IADJOINT, NVAR, &
      IINMODE, IOUTMODE, IPRINT
   INTEGER, DIMENSION(10)                          :: DIM
   INTEGER, DIMENSION(33)                          :: INFO
   DOUBLEPRECISION, DIMENSION(0:1)                 :: T
   DOUBLEPRECISION, DIMENSION(:, :), ALLOCATABLE   :: XL, XU, UL, UU, P, G, BC, STATE, CONTROL, &
      SCONSTRAINTS, DSOLREALTIME
   DOUBLEPRECISION, DIMENSION(7)                   :: TOL
   DOUBLEPRECISION, DIMENSION(:), ALLOCATABLE      :: TAUU, TAUX, TMEASURE, SOL, TIME, PARAMETERS
   DOUBLEPRECISION                                 :: HREALTIME
   ! end of don't change the following declarations
   !------------------------------------------------------------------------------------------------------------

   ! Define user arrays of appropriate length
   INTEGER, DIMENSION(:), ALLOCATABLE              :: IUSER
   DOUBLEPRECISION, DIMENSION(:), ALLOCATABLE      :: USER

   ! define local variables
   INTEGER                         :: idx, jdx, nrSolIter, nrMpcShifts
   ! Variables for the road module
   INTEGER                         :: MODE1, IOUT, STATUS
   INTEGER, DIMENSION(4)           :: IPARAM
   DOUBLEPRECISION, DIMENSION(4)   :: PARAM
   CHARACTER(LEN=20)               :: fileName ! File with (x,y,z,slopel,slopeq,width) for reference line
   CHARACTER(LEN=20)               :: NAME ! Output file with (S,X,Y,Z,SL,SQ,WIDTH,KAPPA,...) after INITROAD
   DOUBLEPRECISION                 :: egoXPos, egoYPos, egoPsi, maxSpeed, kappa_ref
   REAL                            :: cpuBegTime, cpuEndTime

   ! Enter problem specific dimensions and data here
   TOL(4) = 1.0D-12                ! Tolerance state
   TOL(5) = 1.0D+7                 ! Tolerance sensitivities
   TOL(6) = 1.0D-6                 ! finite difference step size
   TOL(7) = 1.0D-4                 ! Tolerance algebraic variable
   ! set output flag for printing details of the optimal control problem
   IPRINT = 0

   ! set dimensions
   DIM(1) = 6                      ! NX number of states
   DIM(2) = 2                      ! NU number of controls
   DIM(3) = 0                      ! NP number of optimization parameters
   DIM(4) = 3                      ! NG number of state constraints
   DIM(5) = 0                      ! NBC number of boundary conditions in BDCOND
   DIM(6) = 16                     ! NGITU number of control grid points
   DIM(7) = 1                      ! NGITX number of shooting nodes
   DIM(8) = 0                      ! NROOT number of switching functions
   DIM(9) = 0                      ! NY number of algebraic variables in x
   DIM(10) = 0                     ! NMEASURE number of measure points xi_i in function H

   ! initial time, length of time interval
   T(0) = 0.0D0                    ! provide initial time t0
   T(1) = 3.0D0                    ! provide length tf-t0

   ! set parameters that control the algorithm
   INFO(1) = 0                     ! IEQ flag equidistant grids (=0) or non-equidistant grids (=1)
   INFO(2) = 11                    ! INTEG integrator, see user's guide for a list
   INFO(3) = 1                     ! ISTEUER control approximation by B-splines
   INFO(4) = 10000                 ! ILIMIT
   INFO(5) = 1                     ! IMETHOD method for gradient calculation
   INFO(6) = 0                     ! IMASS structure of matrix F'_{x'} and M, respectively
   INFO(7) = 0                     ! IITMAT flag for iteration matrix
   INFO(8) = 0                     ! IGJAC flag for jacobian of state constraints
   INFO(9) = 6                     ! ISTREAM output will be printed to output file with this number (6=screen)
   INFO(10) = 1                    ! IFD flag for finite differences
   INFO(11) = 0                    ! IALG computation of consistent initial values for index-1 DAEs

   ! how will the initial guess be provided and where shall the output be written to?
   IINMODE = 0                     ! 0   = initial guess for states in INESTX, for controls in INESTU
   ! <>0 = initial guess in SOL
   IOUTMODE = 2                    ! 1   = output of solution will be written to files
   ! 2   = output of solution will be written to arrays
   !       TIME,STATE,CONTROL,PARAMETERS,SCONSTRAINTS,DSOLREALTIME
   ! 3   = output of solution will be written to both files and variables

   ! Realtime optimization
   IREALTIME = 0                   ! switch for realtime optimization
   NREALTIME = 2                   ! number of realtime parameters
   HREALTIME = 1.0D-3              ! step-size used in finite difference Hessian approximation

   ! Provide realtime parameters in USER(IUSER(I)),I=1,...,NREALTIME, if IREALTIME > 0
   ! ALLOCATE (IUSER(1))
   ! ALLOCATE (USER(1))

   ! switch for adjoint estimation
   IADJOINT = 1                    ! switch for adjoint estimation (1 = yes, 0 = no)

   ! provide measure points xi_i, i=1,...,L, for parameter identification (if DIM(10)>0)
   ALLOCATE (TMEASURE(MAX(1, DIM(10))))

   ! provide box constraints for state at shooting nodes
   ALLOCATE (XL(0:1, MAX(1, DIM(1))))
   ALLOCATE (XU(0:1, MAX(1, DIM(1))))
   ! lower and upper bounds for initial state x(t0)
   XL(0, 1:DIM(1)) = 0.0D0
   XU(0, 1:DIM(1)) = 0.0D0
   ! lower and upper bounds for states at multiple shooting nodes in (t0,tf)
   XL(1, 1:DIM(1)) = -1.0D+20
   XU(1, 1:DIM(1)) = 1.0D+20

   ! provide box constraints for control
   ALLOCATE (UL(-1:1, MAX(1, DIM(2))))
   ALLOCATE (UU(-1:1, MAX(1, DIM(2))))

   ! provide box constraints and initial guess for optimization parameter p
   ALLOCATE (P(0:2, MAX(1, DIM(3))))
   ! P(0,1:DIM(3)) = -1.0D+20                        ! lower bound for optimization parameter
   ! P(1,1:DIM(3)) = 1.0D+20                        ! upper bound for optimization parameter
   ! P(2,1:DIM(3)) = 0.0D+20                        ! initial guess for optimization parameter

   ! provide lower and upper bounds for state constraints g_l<=g(t,x(t),u(t),p)<=g_u, g needs to be provided in NLCSTR
   ALLOCATE (G(0:1, MAX(1, DIM(4))))
   G(0, 1:DIM(4)) = -1.0D+20
   G(1, 1:DIM(4)) = 0.0D0

   ! provide lower and upper bounds for boundary conditions psi_l<=psi(t0,tf,x(t0),x(tf),u(t0),u(tf),p)<=psi_u
   ALLOCATE (BC(0:1, MAX(1, DIM(5))))
   ! BC(0,1) = 0.0D0                        ! lower bound psi_l for boundary condition
   ! BC(1,1) = 0.0D0                        ! upper bound psi_u for boundary condition
   ! BC(0,2) = 0.0D0
   ! BC(1,2) = 0.0D0

   ! If nonequidistant grids are chosen (INFO(1) <> 0) provide control grid and shooting nodes in TAUU and TAUX
   ALLOCATE (TAUU(MAX(1, DIM(6))))
   ALLOCATE (TAUX(MAX(1, DIM(7))))

   !------------------------------------------------------------------------------------------------------------
   ! Don't change the following declarations
   ALLOCATE (SOL(MAX(1, DIM(1)*DIM(7) + DIM(2)*(DIM(6) + INFO(3) - 2) + DIM(3))))
   ALLOCATE (TIME(MAX(1, DIM(6))))
   ALLOCATE (STATE(MAX0(1, DIM(1)), MAX0(1, DIM(6))))
   ALLOCATE (CONTROL(MAX0(1, DIM(2)), MAX0(1, DIM(6))))
   ALLOCATE (PARAMETERS(MAX0(1, DIM(3))))
   ALLOCATE (SCONSTRAINTS(MAX0(1, DIM(4)), MAX0(1, DIM(6))))
   ALLOCATE (DSOLREALTIME(MAX0(1, NREALTIME), MAX0(1, DIM(7)*DIM(1) + DIM(2)*(DIM(6) + INFO(3) - 2) + DIM(3))))
   ! end of don't change the following declarations
   !------------------------------------------------------------------------------------------------------------

   ! Initialize road data
   fileName = 'velCnstr.dat' ! File with [s, v]
   CALL GetNrFileLines(fileName, nrRoadPoints)
   IF (nrRoadPoints <= 0) THEN
      ! Deallocate memory
      DEALLOCATE (XL, XU, UL, UU, P, G, BC, TAUU, TAUX, TMEASURE, SOL, TIME, STATE, CONTROL, PARAMETERS, &
         SCONSTRAINTS, DSOLREALTIME)
      RETURN
   ELSE
      ! Allocate and prepare memory
      CALL InitRoadData(fileName, nrRoadPoints, nrSolIter)
      IF (nrSolIter < 0) THEN
         ! Deallocate memory
         DEALLOCATE (XL, XU, UL, UU, P, G, BC, TAUU, TAUX, TMEASURE, SOL, TIME, STATE, CONTROL, PARAMETERS, &
            SCONSTRAINTS, DSOLREALTIME)
         RETURN
      END IF
   END IF

   ! Initialize the road variables
   MODE1 = 0 ! 0 = simple road model, 1 = advanced road model, 2 = flat road, 3 = road with periodic bumps (C^1 road)
   NAME = "splineFile.dat" ! File with (S,X,Y,Z,SL,SQ,WIDTH,KAPPA,derivatives of splines for x,y,z,sl,sq,width)
   fileName = "roadMat.dat" ! File with (x,y,z,slopel,slopeq,width) for reference line
   IOUT = 0 ! device number for output (6 = screen)
   IPARAM(:) = 0    ! parameters for advanced road model
   PARAM(:) = 0.0D0 ! parameters for advanced road model
   CALL INITROAD(MODE1, NAME, fileName, IOUT, PARAM, IPARAM, STATUS)

   !
   ! MPC LOOP
   !
   nrSolIter = 1
   nrMpcShifts = 2     ! number of steps (n-1) to be accepted in mpc (shift)

   ! Initialize the state and control arrays
   STATE(1:DIM(1), 1:DIM(6)) = 0.0D0
   XL(0, 1:DIM(1)) = 0.0D0
   XU(0, 1:DIM(1)) = 0.0D0
   CONTROL(1:DIM(2), 1:DIM(6)) = 0.0D0

   ! Set lower and upper bounds for controls u(t), t0<=t<=tf
   UL(-1:1, 1) = -0.2D0
   UU(-1:1, 1) = 0.2D0
   UL(-1:1, 2) = -4.0D0
   UU(-1:1, 2) = 3.5D0

   ! Set lower and upper bounds for state constraints
   G(0, 1) = -0.4D0    ! kappa_min
   G(1, 1) = 0.4D0     ! kappa_max
   G(0, 2) = -15.0D0   ! v_min
   G(1, 2) = 0.0D0     ! v_max: v_max - v <= 0
   G(0, 3) = -4.5D0    ! minimum lateral acceleration
   G(1, 3) = 4.5D0     ! maximum lateral acceleration

   ! Open file for writing all OCP data
   fileName = 'optVelTrk.dat'
   OPEN (11, FILE=fileName, STATUS='UNKNOWN')
   fileName = 'roadDataTrk.dat'
   OPEN (22, FILE=fileName, STATUS='UNKNOWN')
   fileName = 'velDataTrk.dat'
   OPEN (33, FILE=fileName, STATUS='UNKNOWN')
   DO
      IF (nrSolIter > 1) THEN
         ! Initialize using data arrays instead of INESTX, INESTU
         IINMODE = 1
         ! Adjust initial time
         T(0) = TIME(nrMpcShifts) ! step time is Time(2)-T(0)
         ! Adjust initial state bounds
         XL(0, 1:DIM(1)) = STATE(1:DIM(1), nrMpcShifts)
         XU(0, 1:DIM(1)) = XL(0, 1:DIM(1))
      END IF

      ! Start timer
      call cpu_time(cpuBegTime)
      ! Call the solver
      CALL OCPIDDAE1(T, XL, XU, UL, UU, P, G, BC, &
         TOL, TAUU, TAUX, TMEASURE, IRES, &
         IREALTIME, NREALTIME, HREALTIME, DSOLREALTIME, &
         IADJOINT, .FALSE., IPRINT, &
         DIM, INFO, IINMODE, SOL, NVAR, IUSER, USER, &
         IOUTMODE, TIME, STATE, CONTROL, PARAMETERS, SCONSTRAINTS)
      ! End timer
      call cpu_time(cpuEndTime)
      cpuEndTime = cpuEndTime - cpuBegTime

      ! Write data in the terminal
      WRITE (*, '(A4,I5,A10,E13.4,A1,E13.4,A8,I5)') 'NR: ', nrSolIter, &
         ' X: [', XL(0, 1), ',', STATE(1, DIM(6)), '] IRES= ', IRES

      ! Write data in external files
      DO jdx = 1, nrMpcShifts - 1
         ! Determine (x,y,psi) of ego vehicle from s-r coordinates at s(t)
         CALL ConvertCrvToCrtCoords(STATE(1, jdx), STATE(2, jdx), STATE(3, jdx), egoXPos, egoYPos, egoPsi)
         ! Write (x,y,psi), states, control, and constraints data in a file
         CALL GetRoadMaxSpeed(STATE(1, jdx), maxSpeed)
         CALL GetRoadCurvature(STATE(1, jdx), kappa_ref)
         WRITE (11, '(18E30.16)') TIME(jdx), egoXPos, egoYPos, egoPsi, (STATE(idx, jdx), idx=1, DIM(1)), &
            (CONTROL(idx, jdx), idx=1, DIM(2)), (SCONSTRAINTS(idx, jdx), idx=1, DIM(4)), &
            maxSpeed, kappa_ref, cpuEndTime
         WRITE (22, '(6E30.16)') egoXPos, egoYPos, 0.0D0, 0.0D0, 0.0D0, 4.0D0
         WRITE (33, '(4E30.16)') TIME(jdx), STATE(1,1), STATE(5,1), maxSpeed
      END DO

      ! Update the iteration index
      nrSolIter = nrSolIter + 1

      ! Stopping criterion
      IF (nrSolIter >= 1000) EXIT
      IF (((ROAD_S(ROAD_NPOINTS) - XL(0, 1)) <= 0.5D0) .AND. (XL(0, 5) <= 0.1D0) .AND. (CONTROL(0, 2) <= 0.0D0)) EXIT
   END DO
   ! Close files
   CLOSE (11)
   CLOSE (22)
   CLOSE (33)
   !
   ! END MPC LOOP
   !

   ! Deallocate memory
   DEALLOCATE (XL, XU, UL, UU, P, G, BC, TAUU, TAUX, TMEASURE, SOL, TIME, STATE, CONTROL, PARAMETERS, SCONSTRAINTS, &
      DSOLREALTIME)
   CALL CLEANROAD()
END PROGRAM optVelTrk

!-------------------------------------------------------------------------
!     Objective Function
!-------------------------------------------------------------------------
SUBROUTINE OBJ(X0, XF, TF, P, V, IUSER, USER)
   IMPLICIT NONE
   INTEGER, DIMENSION(*), INTENT(INOUT)         :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)    :: X0, XF, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT) :: USER
   DOUBLEPRECISION, INTENT(IN)                  :: TF
   DOUBLEPRECISION, INTENT(OUT)                 :: V
   V = XF(6)
   RETURN
END SUBROUTINE OBJ

!-------------------------------------------------------------------------
!     separable part of the objective function
!     objective function value = obj + sum_i=1^nmeasure hfunc(i,...)
!-------------------------------------------------------------------------
SUBROUTINE HFUNC(I, T, X, U, P, HVAL, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                          :: I
   INTEGER, DIMENSION(*), INTENT(INOUT)         :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)    :: X, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT) :: USER
   DOUBLEPRECISION, INTENT(IN)                  :: T
   DOUBLEPRECISION, INTENT(OUT)                 :: HVAL
   HVAL = 0.0D0
   RETURN
END SUBROUTINE HFUNC

!-------------------------------------------------------------------------
!     Differential Equations
!-------------------------------------------------------------------------
SUBROUTINE DAE(T, X, XP, U, P, F, IFLAG, IUSER, USER)
   USE RoadHelper
   USE ROADMODULE
   IMPLICIT NONE
   INTEGER, INTENT(IN)                          :: IFLAG
   INTEGER, DIMENSION(*), INTENT(INOUT)         :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)    :: X, XP, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT) :: USER
   DOUBLEPRECISION, INTENT(IN)                  :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)   :: F
   ! Local variables
   DOUBLEPRECISION                                 :: kappa_ref, tmpVal

   ! Compute curvature at s(t)
   CALL GetRoadCurvature(X(1), kappa_ref)
   ! s'(t) = v(t)*cos(chi(t))/(1-d(t)*kappa_ref(s))
   F(1) = X(5)*DCOS(X(3))/(1 - (X(2)*kappa_ref))
   ! d'(t) = v(t)*sin(chi(t))
   F(2) = X(5)*DSIN(X(3))
   ! chi'(t) = psi'(t)-psi_ref'(t) = v(t)*kappa(s(t)) - s'(t)*kappa_ref(s(t))
   F(3) = (X(5)*X(4)) - (F(1)*kappa_ref)
   ! kappa'(t) = u1(t)
   F(4) = U(1)
   ! v'(t) = a(t) = u2(t)
   F(5) = U(2)

   tmpVal = (ROAD_S(ROAD_NPOINTS) - X(1))/(ROAD_S(ROAD_NPOINTS) - ROAD_S(0))
   ! tmpVal = (roadDataMatrix(nrRoadPoints, 1) - X(1))/(roadDataMatrix(nrRoadPoints, 1) - roadDataMatrix(1, 1))
   ! Path following: min(X(2),X(3)). Reach destination: min((s_f-X(1))/(s_f-s_0)). Comfort: min(U(1),U(2))
   F(6) = (1.0D-1)*(0.25D0*(X(2))**2) + &
      (1.0D-2)*(X(3)**2) + &
      (4.0D-1)*(tmpVal**2) + &
      (1.0D-3)*(U(1)**2) + &
      (2.0D-5)*(U(2)**2)

   ! Add an overall scaling factor
   F(6) = (5.0D-2)*F(6)
   RETURN
END SUBROUTINE DAE

!-------------------------------------------------------------------------
!     Nonlinear Constraints
!-------------------------------------------------------------------------
SUBROUTINE NLCSTR(T, X, U, P, G, IUSER, USER)
   USE RoadSpline
   IMPLICIT NONE
   ! INOUT ARGUMENTS
   INTEGER, DIMENSION(*), INTENT(INOUT)            :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)       :: X, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)    :: USER
   DOUBLEPRECISION, INTENT(IN)                     :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)      :: G
   ! Local variables
   DOUBLEPRECISION                                 :: maxSpeed

   G(1) = X(4) ! kappa(t)
   CALL GetRoadMaxSpeed(X(1), maxSpeed)
   G(2) = (X(5) - maxSpeed) ! v(t) - v_max
   G(3) = X(4)*(X(5)**2) ! a_n(t) = kappa(t) * v(t)^2
   RETURN
END SUBROUTINE NLCSTR

!-------------------------------------------------------------------------
!     JACOBIAN OF Nonlinear Constraints
!-------------------------------------------------------------------------
SUBROUTINE JACNLC(T, X, U, P, NG, GJAC, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                               :: NG
   INTEGER, DIMENSION(*), INTENT(INOUT)              :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)         :: X, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)      :: USER
   DOUBLEPRECISION, INTENT(IN)                       :: T
   DOUBLEPRECISION, DIMENSION(NG, *), INTENT(OUT)    :: GJAC
   RETURN
END SUBROUTINE JACNLC

!-------------------------------------------------------------------------
!     Boundary Conditions
!-------------------------------------------------------------------------
SUBROUTINE BDCOND(T0, TF, X0, XF, U0, UF, P, PSI, IUSER, USER)
   IMPLICIT NONE
   INTEGER, DIMENSION(*), INTENT(INOUT)              :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)         :: X0, XF, U0, UF, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)      :: USER
   DOUBLEPRECISION, INTENT(IN)                       :: T0, TF
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)        :: PSI
   RETURN
END SUBROUTINE BDCOND

!-------------------------------------------------------------------------
!     Initial Estimate STATE VARIABLES
!-------------------------------------------------------------------------
SUBROUTINE INESTX(T, X, IUSER, USER)
   IMPLICIT NONE
   INTEGER, DIMENSION(*), INTENT(INOUT)              :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)      :: USER
   DOUBLEPRECISION, INTENT(IN)                       :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)        :: X
   X(1) = 0.0D0
   X(2) = 0.0D0
   X(3) = 0.0D0
   X(4) = 0.0D0
   X(5) = 0.0D0
   X(6) = 0.0D0
   RETURN
END SUBROUTINE INESTX

!-------------------------------------------------------------------------
!     Initial Estimate CONTROL VARIABLES
!-------------------------------------------------------------------------
SUBROUTINE INESTU(T, U, IBOOR, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                              :: IBOOR
   INTEGER, DIMENSION(*), INTENT(INOUT)             :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)     :: USER
   DOUBLEPRECISION, INTENT(IN)                      :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)       :: U
   U(1) = 0.0D0
   U(2) = 0.0D0
   RETURN
END SUBROUTINE INESTU

!-------------------------------------------------------------------------
!     Mass Matrix
!-------------------------------------------------------------------------
SUBROUTINE MASS(NX, T, X, XP, U, P, MMASS, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                              :: NX
   INTEGER, DIMENSION(*), INTENT(INOUT)             :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)        :: X, XP, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)     :: USER
   DOUBLEPRECISION, INTENT(IN)                      :: T
   DOUBLEPRECISION, DIMENSION(NX, *), INTENT(OUT)   :: MMASS
   MMASS(1:NX, 1) = 1.0D0
   RETURN
END SUBROUTINE MASS

!-------------------------------------------------------------------------
!     Iteration Matrix
!-------------------------------------------------------------------------
SUBROUTINE ITMAT(T, X, XP, U, P, JAC1, JAC2, CJ, NX, NSTAB, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                               :: NX, NSTAB
   INTEGER, DIMENSION(*), INTENT(INOUT)              :: IUSER
   DOUBLEPRECISION, INTENT(IN)                       :: CJ
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)         :: X, XP, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)      :: USER
   DOUBLEPRECISION, INTENT(IN)                       :: T
   DOUBLEPRECISION, DIMENSION(NX, *), INTENT(OUT)    :: JAC1
   DOUBLEPRECISION, DIMENSION(NSTAB, *), INTENT(OUT) :: JAC2
   RETURN
END SUBROUTINE ITMAT

!-------------------------------------------------------------------------
!     Root Functions
!-------------------------------------------------------------------------
SUBROUTINE ROOT(NX, T, X, U, P, NROOT, R, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                               :: NX, NROOT
   INTEGER, DIMENSION(*), INTENT(INOUT)              :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)         :: X, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)      :: USER
   DOUBLEPRECISION, INTENT(IN)                       :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)        :: R
   RETURN
END SUBROUTINE ROOT

!-------------------------------------------------------------------------
!     Jump Function
!-------------------------------------------------------------------------
SUBROUTINE DJUMP(IC, T, XL, UL, P, D, IUSER, USER)
   IMPLICIT NONE
   INTEGER, INTENT(IN)                               :: IC
   INTEGER, DIMENSION(*), INTENT(INOUT)              :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)         :: XL, UL, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)      :: USER
   DOUBLEPRECISION, INTENT(IN)                       :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)        :: D
   RETURN
END SUBROUTINE DJUMP

!-------------------------------------------------------------------------
!   Private Functions
!-------------------------------------------------------------------------
SUBROUTINE GetNrFileLines(fileName, nrDataPoints)
   IMPLICIT NONE
   CHARACTER(*), INTENT(IN)    :: fileName
   INTEGER, INTENT(INOUT)      :: nrDataPoints
   ! Get number of data points
   nrDataPoints = 0
   OPEN (1, FILE=fileName, STATUS='OLD')
   DO
      READ (1, *, ERR=20, END=10)
      nrDataPoints = nrDataPoints + 1
   END DO
10 CLOSE (1)
   IF (nrDataPoints < 1) THEN
      WRITE (*, *) "NOT ENOUGH DATA IN DATA FILE! ROAD COULD NOT BE INITIALIZED. RETURNING ..."
      RETURN
   END IF
20 CONTINUE
   WRITE (*, *) "ERROR READING DATA FILE! ROAD COULD NOT BE INITIALIZED. RETURNING ..."
   RETURN
END SUBROUTINE GetNrFileLines
!-------------------------------------------------------------------------
!   END Private Functions
!-------------------------------------------------------------------------
