! gfortran pathTrackCar.f90 ./libocpiddae1.a ./libfgssqp.a ./libspline.a ./libroadhelper.a
! ./a.out
MODULE RoadRefSpeed
   IMPLICIT NONE
   INTEGER, SAVE :: nrSpeedPoints
   DOUBLEPRECISION, DIMENSION(1000, 4), SAVE :: refSpeedData

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
      nrSpeedPoints = nrDataPoints
      OPEN (1, FILE=dataFileName, STATUS='OLD')
      DO idx = 1, nrSpeedPoints
         READ (1, *, ERR=30) refSpeedData(idx, 1:4)
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
      arcLenTmp = Max(refSpeedData(1, 2), Min(refSpeedData(nrSpeedPoints, 2), arcLen))

      ! Initialize index pointers to check the array data
      ltIdx = 0
      rtIdx = nrSpeedPoints
      ! Note that the algorithm finds the solution in O(log(n)) steps
      DO idx = 1, nrSpeedPoints + 1
         IF (ltIdx > rtIdx) STOP ! Solution found in the previous step, exit
         ! Check the element in the middle of the left/right indices
         arcLenIdx = ltIdx + ((rtIdx - ltIdx)/2)
         IF ((arcLenTmp >= refSpeedData(arcLenIdx, 2)) .AND. (arcLenTmp <= refSpeedData(arcLenIdx + 1, 2))) THEN
            ! Point with the closest arc length was found
            RETURN
         ELSE
            ! Decide which half to check next (left or right)
            IF (arcLen < refSpeedData(arcLenIdx, 2)) THEN
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

   SUBROUTINE GetRoadMaxSpeed(arcLen, refSpeed)
      Use RoadHelper
      Use ROADMODULE
      IMPLICIT NONE
      ! INOUT ARGUMENTS
      DOUBLEPRECISION, INTENT(IN)     :: arcLen
      DOUBLEPRECISION, INTENT(OUT)    :: refSpeed
      ! Local variables
      INTEGER                         :: idx

      ! Find the element with the closest arc length value
      CALL FindClosestArcLength(arcLen, idx)
      ! Convert to the max speed matrix index
      refSpeed = arcLen*(refSpeedData(nrSpeedPoints, 2) - refSpeedData(1, 2))/(ROAD_S(ROAD_NPOINTS) - ROAD_S(0))
      CALL FindArcLengthIdx(refSpeed, idx)

      IF (idx < nrSpeedPoints) THEN
         IF (refSpeedData(idx + 1, 4) == refSpeedData(idx, 4)) THEN
            refSpeed = refSpeedData(idx, 4)
         ELSE
            ! Interpolate to find speed: v = v_2 - ((v_2 - v_1)*(s_2 - s)/(s_2 - s_1))
            refSpeed = (refSpeedData(idx + 1, 2) - refSpeed)/(refSpeedData(idx + 1, 2) - refSpeedData(idx, 2))
            refSpeed = refSpeedData(idx + 1, 4) - (refSpeedData(idx + 1, 4) - refSpeedData(idx, 4))*refSpeed
         END IF
      ELSE
         refSpeed = refSpeedData(nrSpeedPoints, 4)
      END IF
      RETURN
   END SUBROUTINE GetRoadMaxSpeed

   SUBROUTINE GetRoadRefSpeed(arcLen, refSpeed)
      Use RoadHelper
      Use ROADMODULE
      IMPLICIT NONE
      ! INOUT ARGUMENTS
      DOUBLEPRECISION, INTENT(IN)     :: arcLen
      DOUBLEPRECISION, INTENT(OUT)    :: refSpeed
      ! Local variables
      INTEGER                         :: idx

      ! Find the element with the closest arc length value
      CALL FindClosestArcLength(arcLen, idx)
      ! Convert to the max speed matrix index
      refSpeed = arcLen*(refSpeedData(nrSpeedPoints, 2) - refSpeedData(1, 2))/(ROAD_S(ROAD_NPOINTS) - ROAD_S(0))
      CALL FindArcLengthIdx(refSpeed, idx)

      IF (idx < nrSpeedPoints) THEN
         IF (refSpeedData(idx + 1, 3) == refSpeedData(idx, 3)) THEN
            refSpeed = refSpeedData(idx, 3)
         ELSE
            ! Interpolate to find speed: v = v_2 - ((v_2 - v_1)*(s_2 - s)/(s_2 - s_1))
            refSpeed = (refSpeedData(idx + 1, 2) - refSpeed)/(refSpeedData(idx + 1, 2) - refSpeedData(idx, 2))
            refSpeed = refSpeedData(idx + 1, 3) - (refSpeedData(idx + 1, 3) - refSpeedData(idx, 3))*refSpeed
         END IF
      ELSE
         refSpeed = refSpeedData(nrSpeedPoints, 3)
      END IF
      RETURN
   END SUBROUTINE GetRoadRefSpeed

   SUBROUTINE GetNextSpeed(inArcLen, refSpeed)
      Use RoadHelper
      Use ROADMODULE
      IMPLICIT NONE
      ! INOUT ARGUMENTS
      DOUBLEPRECISION, INTENT(IN)     :: inArcLen
      DOUBLEPRECISION, INTENT(OUT)    :: refSpeed
      ! Local variables
      INTEGER                         :: idx
      DOUBLEPRECISION                 :: tmpArcLen, diffArcLen

      ! Find closest arc length in road matrix
      CALL FindClosestArcLength(inArcLen, idx)
      diffArcLen = (inArcLen - ROAD_S(idx))/(ROAD_S(idx + 1) - ROAD_S(idx))
      ! Convert to the max speed matrix arc length
      tmpArcLen = inArcLen*(refSpeedData(nrSpeedPoints, 2) - refSpeedData(1, 2))/(ROAD_S(ROAD_NPOINTS) - ROAD_S(0))
      CALL FindArcLengthIdx(tmpArcLen, idx)
      diffArcLen = (tmpArcLen - refSpeedData(idx, 2))/(refSpeedData(idx + 1, 2) - refSpeedData(idx, 2))
      ! Look 2 seconds into the future
      idx = Min(idx + 10, nrSpeedPoints - 1)
      ! Calculate the new arc length
      tmpArcLen = refSpeedData(idx, 2) + (diffArcLen*(refSpeedData(idx + 1, 2) - refSpeedData(idx, 2)))
      ! Revert to the original arclength matrix
      tmpArcLen = tmpArcLen*(ROAD_S(ROAD_NPOINTS) - ROAD_S(0))/(refSpeedData(nrSpeedPoints, 2) - refSpeedData(1, 2))
      ! Find the corresponding ref speed value
      CALL GetRoadRefSpeed(tmpArcLen, refSpeed)
      RETURN
   END SUBROUTINE GetNextSpeed

   SUBROUTINE GetMinSafeClear(velocity, clearDist)
      IMPLICIT NONE
      DOUBLEPRECISION, INTENT(IN)     :: velocity
      DOUBLEPRECISION, INTENT(OUT)    :: clearDist
      clearDist = 8.0D0 + 1.5D0*velocity ! Max(8.0D0, (1.8D0*velocity))
      RETURN
   END SUBROUTINE GetMinSafeClear

END MODULE RoadRefSpeed

PROGRAM pathTrackCar
   USE ROADMODULE
   USE RoadHelper
   USE RoadRefSpeed
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
   DOUBLEPRECISION                 :: egoXPos, egoYPos, egoPsi, refSpeed, refKappa, maxSpeed, nextRefSpeed
   REAL                            :: cpuBegTime, cpuEndTime
   INTEGER                         :: O_inLn, currStateFSM
   DOUBLEPRECISION                 :: transPhaseVal, s_oth
   ! Variables for the road module
   INTEGER                         :: MODE1, IOUT, STATUS
   INTEGER, DIMENSION(4)           :: IPARAM
   DOUBLEPRECISION, DIMENSION(4)   :: PARAM
   CHARACTER(LEN=20)               :: fileName ! File with (x,y,z,slopel,slopeq,width) for reference line
   CHARACTER(LEN=20)               :: NAME ! Output file with (S,X,Y,Z,SL,SQ,WIDTH,KAPPA,...) after INITROAD

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
   DIM(3) = 3                      ! NP number of optimization parameters
   DIM(4) = 6                      ! NG number of state constraints
   DIM(5) = 0                      ! NBC number of boundary conditions in BDCOND
   DIM(6) = 11                     ! NGITU number of control grid points
   DIM(7) = 1                      ! NGITX number of shooting nodes
   DIM(8) = 0                      ! NROOT number of switching functions
   DIM(9) = 0                      ! NY number of algebraic variables in x
   DIM(10) = 0                     ! NMEASURE number of measure points xi_i in function H

   ! initial time, length of time interval
   T(0) = 0.0D0                    ! provide initial time t0
   T(1) = 2.0D0                    ! provide length tf-t0

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
   ALLOCATE (IUSER(1))
   ALLOCATE (USER(5))

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

   P(0, 1) = -1.0D+20  ! lower bound for clearSf relaxation
   P(1, 1) = 3.5D0     ! upper bound for clearSf relaxation
   P(2, 1) = 0.0D0     ! initial guess for P
   P(0, 2) = -1.0D+20  ! lower bound for eta_kappa relaxation
   P(1, 2) = 0.4D0     ! upper bound for eta_kappa relaxation
   P(2, 2) = 0.0D0     ! initial guess for P
   P(0, 3) = -1.0D+20  ! lower bound for eta_v relaxation
   P(1, 3) = 0.6D0     ! upper bound for eta_v relaxation
   P(2, 3) = 0.0D0     ! initial guess for P

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
   fileName = 'velDataCar.dat' ! File with [s, v]
   CALL GetNrFileLines(fileName, nrSpeedPoints)
   IF (nrSpeedPoints <= 0) THEN
      ! Deallocate memory
      DEALLOCATE (XL, XU, UL, UU, P, G, BC, TAUU, TAUX, TMEASURE, SOL, TIME, STATE, CONTROL, PARAMETERS, &
         SCONSTRAINTS, DSOLREALTIME)
      RETURN
   ELSE
      ! Allocate and prepare memory
      CALL InitRoadData(fileName, nrSpeedPoints, nrSolIter)
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
   fileName = "roadDataCar.dat" ! File with (x,y,z,slopel,slopeq,width) for reference line
   IOUT = 0 ! device number for output (6 = screen)
   IPARAM(:) = 0    ! parameters for advanced road model
   PARAM(:) = 0.0D0 ! parameters for advanced road model
   CALL INITROAD(MODE1, NAME, fileName, IOUT, PARAM, IPARAM, STATUS)

   !
   ! MPC LOOP
   !
   nrSolIter = 1
   nrMpcShifts = 2     ! number of steps (n-1) to be accepted in mpc (shift)

   ! Initialize the user arrays
   IUSER(1) = 10
   USER(1:5) = 0.0D0

   ! Initialize the state and control arrays
   STATE(1:DIM(1), 1:DIM(6)) = 0.0D0
   XL(0, 1:DIM(1)) = 0.0D0
   XU(0, 1:DIM(1)) = 0.0D0
   CONTROL(1:DIM(2), 1:DIM(6)) = 0.0D0

   ! Open file for writing all OCP data
   fileName = 'pathTrackCar.dat'
   OPEN (11, FILE=fileName, STATUS='UNKNOWN')
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

      ! Add virtual traffic light
      IF ((T(0) >= 150.0D0) .AND. (T(0) <= 180.0D0)) THEN
         O_inLn = 1
         s_oth = 1270.0D0
      ELSE
         O_inLn = 0
         s_oth = 1.0D+10
      END IF

      ! Check which state to execute
      currStateFSM = IUSER(1)
      CALL GetRoadMaxSpeed(XL(0, 1), maxSpeed)
      IF (currStateFSM == 10) THEN ! Exit Parking (XP)
         ! Check exit conditions
         IF (((XL(0, 1) - ROAD_S(0)) >= 40.0D0) .AND. (maxSpeed >= 13.5D0)) THEN
            currStateFSM = 20 ! Switch to PF (12)
         ELSE IF (((XL(0, 1) - ROAD_S(0)) >= 40.0D0) .AND. (maxSpeed >= 8.0D0)) THEN
            currStateFSM = 30 ! Switch to PU (13)
         END IF
      ELSE IF (currStateFSM == 20) THEN ! Path Following (PF)
         ! Check exit conditions
         IF ((ROAD_S(ROAD_NPOINTS) - XL(0, 1)) <= 18.0D0) THEN
            currStateFSM = 50 ! Switch to NP (25)
         ELSE IF ((XL(0, 5) <= 8.0D0) .AND. (CONTROL(0, 2) <= 0.0D0) .AND. ((O_inLn > 0) .OR. (maxSpeed <= 8.4D0))) THEN
            currStateFSM = 30 ! Switch to PU (23)
         END IF
      ELSE IF (currStateFSM == 30) THEN ! Pulling Up (PU)
         ! Check exit conditions
         IF ((ROAD_S(ROAD_NPOINTS) - XL(0, 1)) <= 18.0D0) THEN
            currStateFSM = 50 ! Switch to NP (35)
         ELSE IF ((maxSpeed >= 13.5D0) .AND. ((O_inLn <= 0) .OR. (XL(0, 5) >= 13.5D0))) THEN
            currStateFSM = 20 ! Switch to PF (32)
         ELSE IF ((XL(0, 5) <= 0.5D0) .AND. (CONTROL(0, 2) <= 0.0D0) .AND. (s_oth - XL(0, 1) <= 10.0D0)) THEN
            currStateFSM = 40 ! Switch to SS (34)
         END IF
      ELSE IF (currStateFSM == 40) THEN ! Standstill (SS)
         ! Check exit conditions
         IF ((O_inLn <= 0) .OR. ((s_oth - XL(0, 1)) >= 13.0D0)) THEN
            currStateFSM = 30 ! Switch to PU (43)
         END IF
      ELSE IF (currStateFSM == 50) THEN ! Enter Parking (NP)
         ! Check exit conditions
         IF ((ROAD_S(ROAD_NPOINTS) - XL(0, 1)) <= 2.0D0) THEN
            currStateFSM = 60 ! Switch to End
         END IF
      ELSE ! IF (currStateFSM == 60) THEN ! End
         ! Do nothing
      END IF
      IUSER(1) = currStateFSM ! Update FSM current state flag

      ! Solve the problem for the FSM current state
      transPhaseVal = 0.0D0
      USER(1) = 0.0D0
      IF (currStateFSM == 10) THEN ! Exit Parking (XP)
         ! Check transition phases
         IF (((XL(0, 1) - ROAD_S(0)) >= 20.0D0) .AND. (maxSpeed >= 8.5D0)) THEN
            ! [12] Transition phase from XP(1) to PF(2)
            transPhaseVal = 1.0D0 + Exp((0.5D0*XL(0, 1)) - 15.0D0)
            transPhaseVal = 1.0D0/transPhaseVal
            USER(1) = transPhaseVal
            IUSER(1) = 12 ! Temporary switch to transition phase [XP->PF]
            ! [XP->PF] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.6D0 - (-0.2D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 1) = 0.6D0 - (0.2D0*(1.0D0 - transPhaseVal))
            UL(-1:1, 2) = -5.0D0 - (-1.0D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 2) = 4.0D0 - (1.5D0*(1.0D0 - transPhaseVal))
            ! State limits (constraints)
            USER(2) = -0.8D0 - (-0.2D0*(1.0D0 - transPhaseVal)) ! kappa_min
            USER(3) = 0.8D0 - (0.2D0*(1.0D0 - transPhaseVal)) ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 2.7D0 + (10.8D0*(1.0D0 - transPhaseVal)) ! v_max
         ELSE IF (((XL(0, 1) - ROAD_S(0)) >= 20.0D0)) THEN
            ! [13] Transition phase from XP(1) to PU(3)
            transPhaseVal = 1.0D0 + Exp((0.5D0*XL(0, 1)) - 15.0D0)
            transPhaseVal = 1.0D0/transPhaseVal
            USER(1) = transPhaseVal
            IUSER(1) = 13 ! Temporary switch to transition phase [XP->PU]
            ! [XP->PU] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.6D0 - (-0.2D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 1) = 0.6D0 - (0.2D0*(1.0D0 - transPhaseVal))
            UL(-1:1, 2) = -5.0D0 - (-1.0D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 2) = 4.0D0
            ! State limits (constraints)
            USER(2) = -0.8D0 - (-0.2D0*(1.0D0 - transPhaseVal)) ! kappa_min
            USER(3) = 0.8D0 - (0.2D0*(1.0D0 - transPhaseVal)) ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 2.7D0 + (5.3D0*(1.0D0 - transPhaseVal)) ! v_max
         ELSE
            ! [XP] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.6D0
            UU(-1:1, 1) = 0.6D0
            UL(-1:1, 2) = -5.0D0
            UU(-1:1, 2) = 4.0D0
            ! State limits (constraints)
            USER(2) = -0.8D0 ! kappa_min
            USER(3) = 0.8D0 ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 2.7D0 ! v_max
         END IF

      ELSE IF (currStateFSM == 20) THEN ! Path Following (PF)
         ! Check transition phases
         IF ((ROAD_S(ROAD_NPOINTS) - XL(0, 1)) <= 100.0D0) THEN
            ! [25] Transition phase from PF(2) to NP(5)
            transPhaseVal = 1.0D0 + Exp(9.0D0 - (0.15D0*(ROAD_S(ROAD_NPOINTS) - XL(0, 1))))
            transPhaseVal = 1.0D0/transPhaseVal
            USER(1) = transPhaseVal
            IUSER(1) = 25 ! Temporary switch to transition phase [PF->NP]
            ! [PF->NP] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.4D0 - (0.2D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 1) = 0.4D0 - (-0.2D0*(1.0D0 - transPhaseVal))
            UL(-1:1, 2) = -4.0D0 - (1.0D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 2) = 2.5D0 - (-1.5D0*(1.0D0 - transPhaseVal))
            ! State limits (constraints)
            USER(2) = -0.8D0 - (0.2D0*(1.0D0 - transPhaseVal)) ! kappa_min
            USER(3) = 0.8D0 - (-0.2D0*(1.0D0 - transPhaseVal)) ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 13.5D0 - (10.8D0*(1.0D0 - transPhaseVal)) ! v_max
         ELSE IF ((maxSpeed < 13.5D0) .OR. ((O_inLn > 0) .AND. (XL(0, 5) < 13.5D0))) THEN
            ! [23] Transition phase from PF(2) to PU(3)
            transPhaseVal = 1.0D0 + Exp(22.0D0 - (2.0D0*XL(0, 5)))
            transPhaseVal = 1.0D0/transPhaseVal
            USER(1) = transPhaseVal
            IUSER(1) = 23 ! Temporary switch to transition phase [PF->PU]
            ! [PF->PU] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.4D0
            UU(-1:1, 1) = 0.4D0
            UL(-1:1, 2) = -4.0D0
            UU(-1:1, 2) = 2.5D0 - (-1.5D0*(1.0D0 - transPhaseVal))
            ! State limits (constraints)
            USER(2) = -0.8D0 ! kappa_min
            USER(3) = 0.8D0 ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 13.5D0 - (5.5D0*(1.0D0 - transPhaseVal)) ! v_max
         ELSE
            ! [PF] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.4D0
            UU(-1:1, 1) = 0.4D0
            UL(-1:1, 2) = -4.0D0
            UU(-1:1, 2) = 2.5D0
            ! State limits (constraints)
            USER(2) = -0.8D0 ! kappa_min
            USER(3) = 0.8D0 ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 13.5D0 ! v_max
         END IF

      ELSE IF (currStateFSM == 30) THEN ! Pulling Up (PU)
         ! Check transition phases
         IF ((ROAD_S(ROAD_NPOINTS) - XL(0, 1)) <= 60.0D0) THEN
            ! [35] Transition phase from PU(3) to NP(5)
            transPhaseVal = 1.0D0 + Exp(15.D0 - (0.5D0*(ROAD_S(ROAD_NPOINTS) - XL(0, 1))))
            transPhaseVal = 1.0D0/transPhaseVal
            USER(1) = transPhaseVal
            IUSER(1) = 35 ! Temporary switch to transition phase [PF->NP]
            ! [PU->NP] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.4D0 - (0.2D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 1) = 0.4D0 - (-0.2D0*(1.0D0 - transPhaseVal))
            UL(-1:1, 2) = -4.0D0 - (1.0D0*(1.0D0 - transPhaseVal))
            UU(-1:1, 2) = 4.0D0
            ! State limits (constraints)
            USER(2) = -0.8D0 - (0.2D0*(1.0D0 - transPhaseVal)) ! kappa_min
            USER(3) = 0.8D0 - (-0.2D0*(1.0D0 - transPhaseVal)) ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 8.0D0 - (5.3D0*(1.0D0 - transPhaseVal)) ! v_max
         ELSE IF ((maxSpeed > 8.4D0) .AND. ((O_inLn <= 0) .OR. (XL(0, 5) >= 8.0D0))) THEN
            ! [32] Transition phase from PU(3) to PF(2)
            transPhaseVal = 1.0D0 + Exp(22.0D0 - 2.0D0*(XL(0, 5)))
            transPhaseVal = 1.0D0 - (1.0D0/transPhaseVal)
            USER(1) = transPhaseVal
            IUSER(1) = 32 ! Temporary switch to transition phase [PU->PF]
            ! [PU->PF] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.4D0
            UU(-1:1, 1) = 0.4D0
            UL(-1:1, 2) = -4.0D0
            UU(-1:1, 2) = 4.0D0 - (1.5D0*(1.0D0 - transPhaseVal))
            ! State limits (constraints)
            USER(2) = -0.8D0 ! kappa_min
            USER(3) = 0.8D0 ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 8.0D0 + (5.5D0*(1.0D0 - transPhaseVal)) ! v_max
         ELSE
            ! [PU] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
            UL(-1:1, 1) = -0.4D0
            UU(-1:1, 1) = 0.4D0
            UL(-1:1, 2) = -4.0D0
            UU(-1:1, 2) = 4.0D0
            ! State limits (constraints)
            USER(2) = -0.8D0 ! kappa_min
            USER(3) = 0.8D0 ! kappa_max
            USER(4) = 0.0D0 ! v_min
            USER(5) = 8.0D0 ! v_max
         END IF

      ELSE IF (currStateFSM == 40) THEN ! Standstill (SS)
         ! [SS] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
         UL(-1:1, 1) = -0.6D0
         UU(-1:1, 1) = 0.6D0
         UL(-1:1, 2) = -5.0D0
         UU(-1:1, 2) = 4.0D0
         ! State limits (constraints)
         USER(2) = -1.0D0 ! kappa_min
         USER(3) = 1.0D0 ! kappa_max
         USER(4) = 0.0D0 ! v_min
         USER(5) = 2.7D0 ! v_max

      ELSE IF (currStateFSM == 50) THEN ! Enter Parking (NP)
         ! [NP] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
         UL(-1:1, 1) = -0.6D0
         UU(-1:1, 1) = 0.6D0
         UL(-1:1, 2) = -5.0D0
         UU(-1:1, 2) = 4.0D0
         ! State limits (constraints)
         USER(2) = -1.0D0 ! kappa_min
         USER(3) = 1.0D0 ! kappa_max
         USER(4) = 0.0D0 ! v_min
         USER(5) = 2.7D0 ! v_max

      ELSE ! IF (currStateFSM == 60) THEN ! (ND)
         ! [ND] Compute solution with lower and upper bounds for control u(t), t0<=t<=tf
         UL(-1:1, 1) = -0.4D0
         UU(-1:1, 1) = 0.4D0
         UL(-1:1, 2) = -5.0D0
         UU(-1:1, 2) = 4.0D0
         ! State limits (constraints)
         USER(2) = -1.0D0 ! kappa_min
         USER(3) = 1.0D0 ! kappa_max
         USER(4) = 0.0D0 ! v_min
         USER(5) = 2.7D0 ! v_max
      END IF

      ! Update the state constraints
      G(0, 1) = USER(2) ! kappa_min
      G(1, 1) = USER(3) ! kappa_max
      G(0, 2) = USER(4) ! v_min
      G(1, 2) = USER(5) ! v_max

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
         CALL GetRoadRefSpeed(STATE(1, jdx), refSpeed)
         CALL GetRoadMaxSpeed(STATE(1, jdx), maxSpeed)
         CALL GetNextSpeed(STATE(1, jdx), nextRefSpeed)
         CALL GetRoadCurvature(STATE(1, jdx), refKappa)
         WRITE (11, '(25E30.16)') TIME(jdx), egoXPos, egoYPos, egoPsi, (STATE(idx, jdx), idx=1, DIM(1)), &
            (CONTROL(idx, jdx), idx=1, DIM(2)), (SCONSTRAINTS(idx, jdx), idx=1, DIM(4)), &
            refKappa, refSpeed, maxSpeed, nextRefSpeed, cpuEndTime, REAL(currStateFSM), transPhaseVal
      END DO

      IUSER(1) = currStateFSM ! Remember to revert the current state flag (for transition phases)

      ! Update the iteration index
      nrSolIter = nrSolIter + 1

      ! Stopping criterion
      IF (nrSolIter >= 2000) EXIT
      IF ((currStateFSM == 60) .AND. (XL(0, 5) <= 0.1D0)) EXIT
   END DO
   ! Close files
   CLOSE (11)
   !
   ! END MPC LOOP
   !

   ! Deallocate memory
   DEALLOCATE (XL, XU, UL, UU, P, G, BC, TAUU, TAUX, TMEASURE, SOL, TIME, STATE, CONTROL, PARAMETERS, SCONSTRAINTS, &
      DSOLREALTIME)
   DEALLOCATE (IUSER, USER)
   CALL CLEANROAD()
END PROGRAM pathTrackCar

!-------------------------------------------------------------------------
!     Objective Function
!-------------------------------------------------------------------------
SUBROUTINE OBJ(X0, XF, TF, P, V, IUSER, USER)
   Use RoadRefSpeed
   IMPLICIT NONE
   INTEGER, DIMENSION(*), INTENT(INOUT)         :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)    :: X0, XF, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT) :: USER
   DOUBLEPRECISION, INTENT(IN)                  :: TF
   DOUBLEPRECISION, INTENT(OUT)                 :: V
   ! Local variables
   DOUBLEPRECISION                              :: tmpVal, transPhaseVal
   INTEGER                                      :: currStateFSM

   ! Get next reference speed
   CALL GetNextSpeed(X0(1), tmpVal)

   ! Adjust OBJ based on the current FSM state
   currStateFSM = IUSER(1)
   transPhaseVal = USER(1)
   tmpVal = 2.0D0*((XF(5) - tmpVal)/(tmpVal + 0.1D0))**2
   IF (currStateFSM == 10) THEN ! Exit Parking (XP)
      V = XF(6) + (8.0D-1)*tmpVal
   ELSE IF (currStateFSM == 12) THEN ! Transition Phase (XP->PF)
      V = XF(6) + ((8.0D-1) - ((2.0D-1)*(1 - transPhaseVal)))*tmpVal

   ELSE IF (currStateFSM == 13) THEN ! Transition Phase (XP->PU)
      V = XF(6) + ((8.0D-1) - ((4.0D-1)*(1 - transPhaseVal)))*tmpVal

   ELSE IF (currStateFSM == 20) THEN ! Path Following (PF)
      V = XF(6) + (6.0D-1)*tmpVal

   ELSE IF (currStateFSM == 23) THEN ! Transition Phase (PF->PU)
      V = XF(6) + ((6.0D-1) - ((2.0D-1)*(1 - transPhaseVal)))*tmpVal

   ELSE IF (currStateFSM == 25) THEN ! Transition Phase (PF->NP)
      V = XF(6) + ((6.0D-1) - ((2.0D-1)*(1 - transPhaseVal)))*tmpVal

   ELSE IF (currStateFSM == 30) THEN ! Pulling Up (PU)
      V = XF(6) + (4.0D-1)*tmpVal

   ELSE IF (currStateFSM == 32) THEN ! Transition Phase (PU->PF)
      V = XF(6) + ((4.0D-1) + ((2.0D-1)*(1 - transPhaseVal)))*tmpVal

   ELSE IF (currStateFSM == 35) THEN ! Transition Phase (PU->NP)
      V = XF(6) + (4.0D-1)*tmpVal

   ELSE IF (currStateFSM == 40) THEN ! Standstill (SS)
      ! Just min(v) to stop moving
      V = XF(6)

   ELSE IF (currStateFSM == 50) THEN ! Enter Parking (NP)
      V = XF(6) + (4.0D-1)*tmpVal

   ELSE ! IF (currStateFSM == 60) THEN ! End
      ! Just min(v) to stop moving
      V = XF(6)
   END IF

   ! Add an overall scaling factor
   V = (5.0D-1)*V
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
   IMPLICIT NONE
   INTEGER, INTENT(IN)                          :: IFLAG
   INTEGER, DIMENSION(*), INTENT(INOUT)         :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)    :: X, XP, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT) :: USER
   DOUBLEPRECISION, INTENT(IN)                  :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)   :: F
   ! Local variables
   DOUBLEPRECISION                              :: refKappa

   ! s'(t) ~= v(t)
   F(1) = X(5)
   ! d'(t) ~= v(t)*(psi(t) - psi_ref(t))
   F(2) = X(5)*X(3)
   ! psi'(t) = v(t)*kappa(s(t))
   ! psi_ref'(t) = refSpeed(s(t))*refKappa(s(t))
   CALL GetRoadCurvature(X(1), refKappa) ! Compute road curvature at s(t)
   F(3) = X(5)*(X(4) - refKappa)
   ! kappa'(t) = u1(t)
   F(4) = U(1)
   ! v'(t) = a(t) = u2(t)
   F(5) = U(2)

   CALL ExecuteDaeObj(T, X, U, P, F(6), IUSER, USER)
   RETURN
END SUBROUTINE DAE

SUBROUTINE ExecuteDaeObj(T, X, U, P, outVal, IUSER, USER)
   USE RoadRefSpeed
   USE ROADMODULE
   IMPLICIT NONE
   INTEGER, DIMENSION(*), INTENT(INOUT)            :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)       :: X, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)    :: USER
   DOUBLEPRECISION, INTENT(IN)                     :: T
   DOUBLEPRECISION, INTENT(OUT)                    :: outVal
   ! Local variables
   INTEGER                                         :: currStateFSM
   DOUBLE PRECISION                                :: tmpVal, transPhaseVal, refSpeed
   ! Construct the objective function in the last state
   ! Reach destination goals: min(s_f-s) => min(s_f-X(1)) [normalize as (s_f-s)/(s_f-s_0)]
   ! Trajectory following goals: min(d,chi) => min(X(2),X(3))
   ! Min energy consumption goals: min(u) => min(U(1)), min(U(2))
   ! Relaxation for transition phases: min(eta_kappa, eta_v)
   ! Safe following relaxation: min(eta_sf)

   ! Adjust OBJ based on the current FSM state
   currStateFSM = IUSER(1)
   transPhaseVal = USER(1)
   IF (currStateFSM == 10) THEN ! Exit Parking (XP)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         (2.0D-3)*(U(1)**2) + &
         (1.0D-5)*(U(2)**2)
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE IF (currStateFSM == 12) THEN ! Transition Phase (XP->PF)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         ((2.0D-3) + ((2.0D-3)*(1 - transPhaseVal)))*(U(1)**2) + &
         ((1.0D-5) + ((4.0D-5)*(1 - transPhaseVal)))*(U(2)**2)
      ! Add transition objective due to restricting kappa
      outVal = outVal + ((4.0D-1)*P(2)**2) ! eta_kappa
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE IF (currStateFSM == 13) THEN ! Transition Phase (XP->PU)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         ((2.0D-3) + ((2.0D-3)*(1 - transPhaseVal)))*(U(1)**2) + &
         ((1.0D-5) + ((4.0D-5)*(1 - transPhaseVal)))*(U(2)**2)
      ! Add transition objective due to restricting kappa
      outVal = outVal + ((4.0D-1)*P(2)**2) ! eta_kappa

   ELSE IF (currStateFSM == 20) THEN ! Path Following (PF)
      outVal = (1.0D-1)*(0.25D0*(X(2))**2) + &
         (1.0D-2)*(X(3)**2) + &
         (4.0D-3)*(U(1)**2) + &
         (5.0D-5)*(U(2)**2)
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE IF (currStateFSM == 23) THEN ! Transition Phase (PF->PU)
      outVal = (1.0D-1)*(0.25D0*(X(2))**2) + &
         (1.0D-2)*(X(3)**2) + &
         (4.0D-3)*(U(1)**2) + &
         (5.0D-5)*(U(2)**2)
      ! Add transition objective due to restricting v
      outVal = outVal + ((4.0D-1)*P(3)**2) ! eta_v
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE IF (currStateFSM == 25) THEN ! Transition Phase (PF->NP)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         ((4.0D-3) - ((3.0D-3)*(1 - transPhaseVal)))*(U(1)**2) + &
         ((5.0D-5) - ((3.0D-5)*(1 - transPhaseVal)))*(U(2)**2)
      ! Add transition objective due to restricting v
      outVal = outVal + ((4.0D-1)*P(3)**2) ! eta_v
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE IF (currStateFSM == 30) THEN ! Pulling Up (PU)
      outVal = (1.0D-1)*(0.25D0*(X(2))**2) + &
         (1.0D-2)*(X(3)**2) + &
         (4.0D-3)*(U(1)**2) + &
         (5.0D-5)*(U(2)**2)
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-3)*tmpVal**2)

   ELSE IF (currStateFSM == 32) THEN ! Transition Phase (PU->PF)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         (4.0D-3)*(U(1)**2) + &
         (5.0D-5)*(U(2)**2)
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-4)*tmpVal**2)

   ELSE IF (currStateFSM == 35) THEN ! Transition Phase (PU->NP)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         ((4.0D-3) - ((3.0D-3)*(1 - transPhaseVal)))*(U(1)**2) + &
         ((5.0D-5) - ((3.0D-5)*(1 - transPhaseVal)))*(U(2)**2)
      ! Add transition objective due to restricting v
      outVal = outVal + ((4.0D-1)*P(3)**2) ! eta_v
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE IF (currStateFSM == 40) THEN ! Standstill (SS)
      ! Just min(v) to stop moving
      outVal = (2.0D-1)*(X(5)**2) + &
         (1.0D-2)*(U(1)**2) + &
         (1.0D-4)*(U(2)**2)

   ELSE IF (currStateFSM == 50) THEN ! Enter Parking (NP)
      outVal = (1.0D-1)*(0.25D0*(X(2)**2)) + &
         (1.0D-2)*(X(3)**2) + &
         (1.0D-3)*(U(1)**2) + &
         (2.0D-5)*(U(2)**2)
      ! Add safe following relaxation
      outVal = outVal + ((8.0D-4)*P(1)**2) ! ((2.0D-5)*tmpVal**2)

   ELSE ! IF (currStateFSM == 60) THEN ! End
      ! Just min(v) to stop moving
      outVal = (2.0D-1)*(X(5)**2) + &
         (1.0D-2)*(U(1)**2) + &
         (1.0D-4)*(U(2)**2)
   END IF
   RETURN
END SUBROUTINE ExecuteDaeObj

!-------------------------------------------------------------------------
!     Nonlinear Constraints
!-------------------------------------------------------------------------
SUBROUTINE NLCSTR(T, X, U, P, G, IUSER, USER)
   USE RoadRefSpeed
   IMPLICIT NONE
   ! INOUT ARGUMENTS
   INTEGER, DIMENSION(*), INTENT(INOUT)            :: IUSER
   DOUBLEPRECISION, DIMENSION(*), INTENT(IN)       :: X, U, P
   DOUBLEPRECISION, DIMENSION(*), INTENT(INOUT)    :: USER
   DOUBLEPRECISION, INTENT(IN)                     :: T
   DOUBLEPRECISION, DIMENSION(*), INTENT(OUT)      :: G
   ! Local variables
   DOUBLEPRECISION                                 :: tmpVal
   ! Curvature constraints kappa_min <= kappa(t) <= kappa_max
   ! Speed constraints v_min <= v(t) <= v_max
   ! Lane constraints clearLN on d(t)
   ! Safe following constraint clearSf

   ! Adjust constraints as per the current FSM state
   IF (IUSER(1) == 10) THEN ! Exit Parking (XP)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 12) THEN ! Transition Phase (XP->PF)
      G(1) = X(4) - P(2) ! kappa(t) + eta_kappa
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 13) THEN ! Transition Phase (XP->PU)
      G(1) = X(4) - P(2) ! kappa(t) + eta_kappa
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 20) THEN ! Path Following (PF)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 23) THEN ! Transition Phase (PF->PU)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) - P(3) ! v(t) + eta_v
   ELSE IF (IUSER(1) == 25) THEN ! Transition Phase (PF->NP)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) - P(3) ! v(t) + eta_v
   ELSE IF (IUSER(1) == 30) THEN ! Pulling Up (PU)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 32) THEN ! Transition Phase (PU->PF)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 35) THEN ! Transition Phase (PU->NP)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) - P(3) ! v(t) + eta_v
   ELSE IF (IUSER(1) == 40) THEN ! Standstill (SS)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   ELSE IF (IUSER(1) == 50) THEN ! Enter Parking (NP)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   ELSE ! IF (IUSER(1) == 60) THEN ! End (ND)
      G(1) = X(4) ! kappa(t)
      G(2) = X(5) ! v(t)
   END IF

   ! clear_LN for (3) disks of radius (1) in a single lane of width (2) d^2 <= (w_LN/2)^2 - r^2
   G(3) = (X(2))**2 - 0.4D0
   G(4) = (X(2) + 1.4D0*(X(3)))**2 - 0.4D0
   G(5) = (X(2) + 2.8D0*(X(3)))**2 - 0.4D0

   ! Add safe following constraint
   IF ((T >= 150.0D0) .AND. (T <= 180.0D0)) THEN
      CALL GetMinSafeClear(X(5), tmpVal)
      G(6) = ((tmpVal - (1270.0D0 - X(1))) - P(1))
   ELSE
      G(6) = 0.0D0
   END IF
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
