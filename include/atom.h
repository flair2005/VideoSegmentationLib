/*
 * atom.h
 *
 *  Created on: 29 Oct 2015
 *      Author: martin
 */

#ifndef ATOM_H_
#define ATOM_H_

#include "segment.h"

namespace videoseg {

class Atom {


public:
	Atom();
	Atom(Segment* segment);
	virtual ~Atom();

	double similarity(const Atom& other) const;

	int id;
	int id_matched_to;
	Segment* segment_;

	static int static_id;
};

} /* namespace videoseg */

#endif /* ATOM_H_ */
