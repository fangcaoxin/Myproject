#include <vector>

using namespace std;
struct Marker {
	int image;
	int track;
	double x, y;
	double weight;
};

class Tracks {
public:
	Tracks() { }

	// Copy constructor for a tracks object.
	Tracks(const Tracks &other);

	/// Construct a new tracks object using the given markers to start.
	explicit Tracks(const vector<Marker> &markers);

	/*!
	Inserts a marker into the set. If there is already a marker for the given
	\a image and \a track, the existing marker is replaced. If there is no
	marker for the given \a image and \a track, a new one is added.

	\a image and \a track are the keys used to retrieve the markers with the
	other methods in this class.

	\a weight is used by bundle adjustment and weight means how much the
	track affects on a final solution.

	\note To get an identifier for a new track, use \l MaxTrack() + 1.
	*/
	// TODO(sergey): Consider using InsetWeightedMarker istead of using
	//               stupid default value?
	void Insert(int image, int track, double x, double y, double weight = 1.0);

	/// Returns all the markers.
	vector<Marker> AllMarkers() const;

	/// Returns all the markers belonging to a track.
	vector<Marker> MarkersForTrack(int track) const;

	/// Returns all the markers visible in \a image.
	vector<Marker> MarkersInImage(int image) const;

	/// Returns all the markers visible in \a image1 and \a image2.
	vector<Marker> MarkersInBothImages(int image1, int image2) const;

	/*!
	Returns the markers in \a image1 and \a image2 which have a common track.

	This is not the same as the union of the markers in \a image1 and \a
	image2; each marker is for a track that appears in both images.
	*/
	vector<Marker> MarkersForTracksInBothImages(int image1, int image2) const;

	/// Returns the marker in \a image belonging to \a track.
	Marker MarkerInImageForTrack(int image, int track) const;

	/// Removes all the markers belonging to \a track.
	void RemoveMarkersForTrack(int track);

	/// Removes the marker in \a image belonging to \a track.
	void RemoveMarker(int image, int track);

	/// Returns the maximum image identifier used.
	int MaxImage() const;

	/// Returns the maximum track identifier used.
	int MaxTrack() const;

	/// Returns the number of markers.
	int NumMarkers() const;

private:
	vector<Marker> markers_;
};
