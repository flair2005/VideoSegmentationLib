/*
 * atom.h
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

#ifndef ATOM_H_
#define ATOM_H_

#include "segment.h"


//distance in pixels
#define MAX_ALLOWED_DISTANCE 50

namespace videoseg {

class Atom {


public:
	Atom();
	Atom(Segment* segment);
	Atom(const Atom& other); // user-defined copy cto
	virtual ~Atom();

	double similarity(const Atom* other) const;

	const Point2i& getCenter() const;

	int id;
	int id_matched_to;
	Segment* segment_;
	double angle;


	static int static_id;
};

} /* namespace videoseg */

#endif /* ATOM_H_ */
