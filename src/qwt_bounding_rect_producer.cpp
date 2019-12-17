/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 2019  Povilas Kanapickas <povilas@radix.lt>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_bounding_rect_producer.h"
#include "qwt_series_data.h"
#include "qwt_point_polar.h"

// don't duplicate vtables in every translation unit
QwtAbstractBoundingRectProducer::~QwtAbstractBoundingRectProducer() {}

template<class T>
QwtBoundingRectProducer<T>::QwtBoundingRectProducer( QwtSeriesData<T>* series ) :
    d_series( series )
{
}

template<class T>
QwtBoundingRectProducer<T>::~QwtBoundingRectProducer() {}

template<class T>
size_t QwtBoundingRectProducer<T>::dataSize() const { return d_series->size(); }

template<class T>
QRectF QwtBoundingRectProducer<T>::sampleRect( size_t i ) const
{
    return qwtBoundingRect( d_series->sample( i ) );
}

template class QwtBoundingRectProducer<QPointF>;
template class QwtBoundingRectProducer<QwtPoint3D>;
template class QwtBoundingRectProducer<QwtPointPolar>;
template class QwtBoundingRectProducer<QwtIntervalSample>;
template class QwtBoundingRectProducer<QwtSetSample>;
template class QwtBoundingRectProducer<QwtOHLCSample>;
