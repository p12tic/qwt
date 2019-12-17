/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 2019  Povilas Kanapickas <povilas@radix.lt>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#ifndef QWT_BOUNDING_RECT_PRODUCER_H
#define QWT_BOUNDING_RECT_PRODUCER_H

#include "qwt_global.h"
#include "qwt_series_data.h"
#include <qrect.h>

/*!
    \brief An interface for producing bounding rectangles for each of a sequence of values.

    This class is used as a data source for QwtBoundingRectCache.

    \sa QwtBoundingRectCache
*/
class QWT_EXPORT QwtAbstractBoundingRectProducer
{
public:
    //! Destructor
    virtual ~QwtAbstractBoundingRectProducer();

    //! Returns the number of samples in the source data
    virtual size_t dataSize() const = 0;

    //! Returns the bounding rectangle for specific sample
    virtual QRectF sampleRect( size_t i ) const = 0;
};

template<class T>
class QWT_EXPORT QwtBoundingRectProducer : public QwtAbstractBoundingRectProducer
{
public:
    /*!
       \brief Constructor
       \param series Data
       Constructs a QwtBoundingRectProducer to refer to data identified by series.
     */
    QwtBoundingRectProducer( QwtSeriesData<T>* series );
    ~QwtBoundingRectProducer() QWT_OVERRIDE;

    size_t dataSize() const QWT_OVERRIDE;
    QRectF sampleRect( size_t i ) const QWT_OVERRIDE;

private:
    QwtSeriesData<T>* d_series;
};

extern template class QwtBoundingRectProducer<QPointF>;
extern template class QwtBoundingRectProducer<QwtPoint3D>;
extern template class QwtBoundingRectProducer<QwtPointPolar>;
extern template class QwtBoundingRectProducer<QwtIntervalSample>;
extern template class QwtBoundingRectProducer<QwtSetSample>;
extern template class QwtBoundingRectProducer<QwtOHLCSample>;

#endif
