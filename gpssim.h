#ifndef GPSSIM_H
#define GPSSIM_H


#ifndef TRUE
#define TRUE (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

/*! \brief Maximum length of a line in a text file (RINEX, motion) */
#define MAX_CHAR (100)

/*! \brief Maximum number of satellites in RINEX file */
#define MAX_SAT (32)

/*! \brief Maximum number of channels we simulate */
#define MAX_CHAN (12)

/*! \brief Maximum number of user motion points */
#define USER_MOTION_SIZE (864000) // for 24 hours at 10Hz

/*! \brief Number of subframes */
#define N_SBF (5) // 5 subframes per frame

/*! \brief Number of words per subframe */
#define N_DWRD_SBF (10) // 10 word per subframe

/*! \brief Number of words */
#define N_DWRD ((N_SBF + 1) * N_DWRD_SBF) // Subframe word buffer size

#define N_SBF_PAGE (3 + 2 * 25) // Subframes 1 to 3 and 25 pages of subframes 4 and 5
#define MAX_PAGE (25)

/*! \brief C/A code sequence length */
#define CA_SEQ_LEN (1023)

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5 0.03125
#define POW2_M11 4.8828125e-4
#define POW2_M19 1.9073486328125e-6
#define POW2_M20 9.5367431640625e-7
#define POW2_M21 4.76837158203125e-7
#define POW2_M23 1.192092895507813e-7
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M38 3.637978807091713e-12
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

// Conventional values employed in GPS ephemeris model (ICD-GPS-200)
#define GM_EARTH 3.986005e14
#define OMEGA_EARTH 7.2921151467e-5
#define PI 3.1415926535898

#define WGS84_RADIUS 6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426

#define R2D 57.2957795131

#define SPEED_OF_LIGHT 2.99792458e8
#define LAMBDA_L1 0.190293672798365

/*! \brief GPS L1 Carrier frequency */
#define CARR_FREQ (1575.42e6)
/*! \brief C/A code frequency */
#define CODE_FREQ (1.023e6)
#define CARR_TO_CODE (1.0 / 1540.0)

// Sampling data format
#define SC01 (1)
#define SC08 (8)
#define SC16 (16)

#define EPHEM_ARRAY_SIZE (13) // for daily GPS broadcast ephemers file (brdc)

/*! \brief Structure representing GPS time */
struct gpstime_t
{
	int week;   /*!< GPS week number (since January 1980) */
	double sec; /*!< second inside the GPS \a week */
};

/*! \brief Structure repreenting UTC time */
struct datetime_t
{
	int y;		/*!< Calendar year */
	int m;		/*!< Calendar month */
	int d;		/*!< Calendar day */
	int hh;		/*!< Calendar hour */
	int mm;		/*!< Calendar minutes */
	double sec; /*!< Calendar seconds */
};

struct almanac_t
{
	gpstime_t toa; // Time of applicability
	int id;
	int health;
	double ecc;	// Eccentricity
	double inc0;   // Inclination
	double omgdot; // Rate of right ascention
	double sqrta;  // SQRT(A)
	double omg0;   // Right ascention at week
	double aop;	// Argument of perigee
	double m0;	 // Mean anomary
	double af0;
	double af1;
};

/*! \brief Structure representing ephemeris of a single satellite */
struct ephem_t
{
	int vflg; /*!< Valid Flag */
	datetime_t t;
	gpstime_t toc; /*!< Time of Clock */
	gpstime_t toe; /*!< Time of Ephemeris */
	int iodc;	  /*!< Issue of Data, Clock */
	int iode;	  /*!< Isuse of Data, Ephemeris */
	double deltan; /*!< Delta-N (radians/sec) */
	double cuc;	/*!< Cuc (radians) */
	double cus;	/*!< Cus (radians) */
	double cic;	/*!< Correction to inclination cos (radians) */
	double cis;	/*!< Correction to inclination sin (radians) */
	double crc;	/*!< Correction to radius cos (meters) */
	double crs;	/*!< Correction to radius sin (meters) */
	double ecc;	/*!< e Eccentricity */
	double sqrta;  /*!< sqrt(A) (sqrt(m)) */
	double m0;	 /*!< Mean anamoly (radians) */
	double omg0;   /*!< Longitude of the ascending node (radians) */
	double inc0;   /*!< Inclination (radians) */
	double aop;
	double omgdot; /*!< Omega dot (radians/s) */
	double idot;   /*!< IDOT (radians/s) */
	double af0;	/*!< Clock offset (seconds) */
	double af1;	/*!< rate (sec/sec) */
	double af2;	/*!< acceleration (sec/sec^2) */
	double tgd;	/*!< Group delay L2 bias */
	// Working variables follow
	double n;		/*!< Mean motion (Average angular velocity) */
	double sq1e2;   /*!< sqrt(1-e^2) */
	double A;		/*!< Semi-major axis */
	double omgkdot; /*!< OmegaDot-OmegaEdot */
};

struct ionoutc_t
{
	int enable;
	int vflg;
	double alpha0, alpha1, alpha2, alpha3;
	double beta0, beta1, beta2, beta3;
	double A0, A1;
	int dtls, tot, wnt;
	int dtlsf, dn, wnlsf;
};

struct range_t
{
	gpstime_t g;
	double range; // pseudorange
	double rate;
	double d; // geometric distance
	double azel[2];
	double iono_delay;
};

/*! \brief Structure representing a Channel */
struct channel_t
{
	int prn;								   /*< PRN Number */
	int ca[CA_SEQ_LEN];						   /*< C/A Sequence */
	double f_carr;							   /*< Carrier frequency */
	double f_code;							   /*< Code frequency */
	unsigned int carr_phase;				   /*< Carrier phase */
	int carr_phasestep;						   /*< Carrier phasestep */
	double code_phase;						   /*< Code phase */
	gpstime_t g0;							   /*!< GPS time at start */
	unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]; /*!< current subframe */
	unsigned long dwrd[N_DWRD];				   /*!< Data words of sub-frame */
	int ipage;
	int iword;   /*!< initial word */
	int ibit;	/*!< initial bit */
	int icode;   /*!< initial code */
	int dataBit; /*!< current data bit */
	int codeCA;  /*!< current C/A code */
	double azel[2];
	range_t rho0;
};

//////////////////////////////////////////////////////////////////////////////

/*! \brief Subtract two vectors of double
 *  \param[out] y Result of subtraction
 *  \param[in] x1 Minuend of subtracion
 *  \param[in] x2 Subtrahend of subtracion
 */
void subVect(double *y, const double *x1, const double *x2);

/*! \brief Compute Norm of Vector
 *  \param[in] x Input vector
 *  \returns Length (Norm) of the input vector
 */
double normVect(const double *x);

/*! \brief Compute dot-product of two vectors
 *  \param[in] x1 First multiplicand
 *  \param[in] x2 Second multiplicand
 *  \returns Dot-product of both multiplicands
 */
double dotProd(const double *x1, const double *x2);

/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN nuber of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
void codegen(int *ca, int prn);

/*! \brief Convert a UTC date into a GPS date
 *  \param[in] t input date in UTC form
 *  \param[out] g output date in GPS form
 */
void date2gps(const datetime_t *t, gpstime_t *g);

void gps2date(const gpstime_t *g, datetime_t *t);

/*! \brief Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Heighth
 *  \param[in] xyz Input Array of X, Y and Z ECEF coordinates
 *  \param[out] llh Output Array of Latitude, Longitude and Height
 */
void xyz2llh(const double *xyz, double *llh);

/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
void llh2xyz(const double *llh, double *xyz);

/*! \brief Compute the intermediate matrix for LLH to ECEF
 *  \param[in] llh Input position in Latitude-Longitude-Height format
 *  \param[out] t Three-by-Three output matrix
 */
void ltcmat(const double *llh, double t[3][3]);

/*! \brief Convert Earth-centered Earth-Fixed to ?
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
void ecef2neu(const double *xyz, double t[3][3], double *neu);

/*! \brief Convert North-Eeast-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
void neu2azel(double *azel, const double *neu);

/*! \brief Compute Satellite position, velocity and clock at given time
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at which position is to be computed
 *  \param[out] pos Computed position (vector)
 *  \param[out] vel Computed velociy (vector)
 *  \param[clk] clk Computed clock
 */
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk);

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[out] sbf Array of five sub-frames, 10 long words each
 */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, const almanac_t *alm, unsigned long sbf[N_SBF_PAGE][N_DWRD_SBF]);

/*! \brief Count number of bits set to 1
 *  \param[in] v long word in whihc bits are counted
 *  \returns Count of bits set to 1
 */
unsigned long countBits(unsigned long v);

/*! \brief Compute the Checksum for one given word of a subframe
 *  \param[in] source The input data
 *  \param[in] nib Does this word contain non-information-bearing bits?
 *  \returns Computed Checksum
 */
unsigned long computeChecksum(unsigned long source, int nib);

/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */
int replaceExpDesignator(char *str, int len);

/// decrease gps time.
double subGpsTime(gpstime_t g1, gpstime_t g0);

/// increase gps time.
gpstime_t incGpsTime(gpstime_t g0, double dt);

int readAlmanac(almanac_t alm[MAX_SAT], const char *fname);

/*! \brief Read Ephemersi data from the RINEX Navigation file */
/*  \param[out] eph Array of Output SV ephemeris data
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname);

double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel);

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[]);

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in dt delta-t (time difference) in seconds
 */
void computeCodePhase(channel_t *chan, range_t rho1, double dt);

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of ECEF vectors for user motion
 *  \param[[in] filename File name of the text input file
 *  \returns Number of user data motion records read, -1 on error
 */

//int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename)
int readUserMotion(double **xyz, const char *filename);

//int readNmeaGGA(double xyz[USER_MOTION_SIZE][3], const char *filename)
int readNmeaGGA(double **xyz, const char *filename);

int generateNavMsg(gpstime_t g, channel_t *chan, int init);

int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel);

int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, almanac_t *alm, gpstime_t grx, double *xyz, double elvMask);

void *gps_task(void *arg);

#endif
