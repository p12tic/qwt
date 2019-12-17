/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 2019  Povilas Kanapickas <povilas@radix.lt>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_BOUNDING_RECT_CACHE_H
#define QWT_BOUNDING_RECT_CACHE_H

#include "qwt_global.h"
#include "qwt_bounding_rect_producer.h"

/*!
    \brief An efficient cache of bounding rect values for series data

    This implements an efficient O(log(n)) cache for bounding rectangle values.

    \sa QwtSeriesData
*/
class QWT_EXPORT QwtBoundingRectCache
{
public:
    QwtBoundingRectCache();
    ~QwtBoundingRectCache();

    /*!
        \brief Builds the cache

        \param rectProducer The producer to retrieve the bounding rectangle data from
        \param orientation The orientation of the input data.
     */
    void build( QwtAbstractBoundingRectProducer* rectProducer, Qt::Orientation orientation );

    /*!
        \brief Retrieves the bounding rect for the data in the range [from; to]

        The cache must be built using the build() method.

        \param fromValue The beginning of the range to scan
        \param toValue The end of the range to scan
        \return The bounding rectangle.
     */
    QRectF boundingRect( double fromValue, double toValue ) const;

private:
    QRectF convertRectToOrientation( const QRectF& rect ) const;

    class PrivateData;
    PrivateData* d_data;
};

#endif
