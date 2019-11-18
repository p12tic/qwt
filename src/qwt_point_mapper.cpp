/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 1997   Josef Wilgen
 * Copyright (C) 2002   Uwe Rathmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_point_mapper.h"
#include "qwt_scale_map.h"
#include "qwt_pixel_matrix.h"
#include "qwt_series_data.h"
#include "qwt_math.h"

#include <qpolygon.h>
#include <qimage.h>
#include <qpen.h>
#include <qpainter.h>

#include <qthread.h>
#include <qfuture.h>
#include <qnumeric.h>
#include <qtconcurrentrun.h>

#if !defined(QT_NO_QFUTURE)
#define QWT_USE_THREADS 1
#endif

static QRectF qwtInvalidRect( 0.0, 0.0, -1.0, -1.0 );

static inline int qwtRoundValue( double value )
{
    return qRound( value );
}

static inline double qwtRoundValueF( double value )
{
#if 1
    // MS Windows and at least IRIX does not have C99's nearbyint() function
    return ( value >= 0.0 ) ? std::floor( value + 0.5 ) : std::ceil( value - 0.5 );
#else
    return nearbyint( value );
#endif
}

static Qt::Orientation qwtProbeOrientation(
    const QwtSeriesData<QPointF> *series, int from, int to )
{
    if ( to - from < 20 )
    {
        // not enough points to "have an orientation"
        return Qt::Horizontal;
    }

    const double x0 = series->sample( from ).x();
    const double xn = series->sample( to ).x();

    if ( x0 == xn )
        return Qt::Vertical;

    const int step = ( to - from ) / 10;
    const bool isIncreasing = xn > x0;

    double x1 = x0;
    for ( int i = from + step; i < to; i += step )
    {
        const double x2 = series->sample( i ).x();
        if ( x2 != x1 )
        {
            if ( ( x2 > x1 ) != isIncreasing )
                return Qt::Vertical;
        }

        x1 = x2;
    }

    return Qt::Horizontal;
}

namespace
{
    template <class MapperOutput, class Point>
    class QwtPolygonQuadrupelX
    {
    public:
        inline void start( int x, int y )
        {
            x0 = x;
            y1 = yMin = yMax = y2 = y;
        }

        inline bool append( int x, int y )
        {
            if ( x0 != x )
                return false;

            if ( y < yMin )
                yMin = y;
            else if ( y > yMax )
                yMax = y;

            y2 = y;

            return true;
        }

        inline void flush( MapperOutput &mapperOutput )
        {
            appendTo( y1, mapperOutput );

            if ( y2 > y1 )
                qSwap( yMin, yMax );

            if ( yMax != y1 )
                appendTo( yMax, mapperOutput );

            if ( yMin != yMax )
                appendTo( yMin, mapperOutput );

            if ( y2 != yMin )
                appendTo( y2, mapperOutput );
        }

    private:
        inline void appendTo( int y, MapperOutput &mapperOutput )
        {
            mapperOutput.current().append( Point( x0, y ) );
        }

    private:
        int x0, y1, yMin, yMax, y2;
    };

    template <class MapperOutput, class Point>
    class QwtPolygonQuadrupelY
    {
    public:
        inline void start( int x, int y )
        {
            y0 = y;
            x1 = xMin = xMax = x2 = x;
        }

        inline bool append( int x, int y )
        {
            if ( y0 != y )
                return false;

            if ( x < xMin )
                xMin = x;
            else if ( x > xMax )
                xMax = x;

            x2 = x;

            return true;
        }

        inline void flush( MapperOutput &mapperOutput )
        {
            appendTo( x1, mapperOutput );

            if ( x2 > x1 )
                qSwap( xMin, xMax );

            if ( xMax != x1 )
                appendTo( xMax, mapperOutput );

            if ( xMin != xMax )
                appendTo( xMin, mapperOutput );

            if ( x2 != xMin )
                appendTo( x2, mapperOutput );
        }

    private:
        inline void appendTo( int x, MapperOutput &mapperOutput )
        {
            mapperOutput.current().append( Point( x, y0 ) );
        }

        int y0, x1, xMin, xMax, x2;
    };

    template <class Polygon, class Point>
    class QwtMapperOutputDontSkip
    {
    public:
        typedef Polygon PolygonType;
        typedef Point PointType;

        static const bool isNone = true;

        QwtMapperOutputDontSkip(Polygon& polygon) : m_polygon(polygon) {}

        bool handleSkip( const QPointF& )
        {
            return false;
        }

        bool handleSkip( const QPointF&, Point*&, int&, int )
        {
            return false;
        }

        Polygon& current() { return m_polygon; }

        const Polygon& current() const { return m_polygon; }
    private:
        Polygon& m_polygon;
    };

    template <class Polygon, class Point>
    class QwtMapperOutputOmitNaN
    {
    public:
        typedef Polygon PolygonType;
        typedef Point PointType;

        static const bool isNone = false;

        QwtMapperOutputOmitNaN(Polygon& polygon) : m_polygon(polygon) {}

        bool handleSkip(const QPointF& point)
        {
            return ( qIsNaN( point.x() ) || qIsNaN( point.y() ) );
        }

        bool handleSkip(const QPointF& point, Point*&, int&, int )
        {
            return handleSkip( point );
        }

        Polygon& current() { return m_polygon; }

        const Polygon& current() const { return m_polygon; }
    private:
        Polygon& m_polygon;
    };

    template <class Polygon, class Point>
    class QwtMapperOutputMultipleDontSkip
    {
    public:
        typedef Polygon PolygonType;
        typedef Point PointType;

        static const bool isNone = false;

        QwtMapperOutputMultipleDontSkip(QVector<Polygon>& polygons) :
            m_polygons(polygons)
        {
            m_polygons.push_back(Polygon());
        }

        ~QwtMapperOutputMultipleDontSkip()
        {
        }

        bool handleSkip( const QPointF& point )
        {
            return ( qIsNaN( point.x() ) || qIsNaN( point.y() ) );
        }

        bool handleSkip( const QPointF& point, Point*&, int&, int )
        {
            return handleSkip( point );
        }

        Polygon& current() { return m_polygons.back(); }

        const Polygon& current() const { return m_polygons.back(); }

    private:
        QVector<Polygon>& m_polygons;
    };

    template <class Polygon, class Point>
    class QwtMapperOutputMultipleSplitNaNToPolygons
    {
    public:
        typedef Polygon PolygonType;
        typedef Point PointType;

        static const bool isNone = false;

        QwtMapperOutputMultipleSplitNaNToPolygons( QVector<Polygon>& polygons ) :
            m_polygons( polygons )
        {
            if ( m_polygons.empty() )
                m_polygons.push_back( Polygon() );
        }

        ~QwtMapperOutputMultipleSplitNaNToPolygons()
        {
            // remove last polygon if it's empty, but always leave at least one polygon
            if ( m_polygons.back().empty() && m_polygons.size() > 2 )
                m_polygons.pop_back();
        }

        bool handleSkip( const QPointF& point )
        {
            bool isNaN = ( qIsNaN( point.x() ) || qIsNaN( point.y() ) );
            if ( isNaN && !m_polygons.back().empty())
            {
                m_polygons.push_back( Polygon() );
            }
            return isNaN;
        }

        // this overload should be used in cases when the caller resizes
        // the current() polygon to excessive size to avoid reallocations,
        // pushes data into the polygon using the data() pointer and then
        // resizes the current() polygon at the end to the actual number
        // of pushed points. The function updates the cached data() pointer
        // and the number of elements pushed to the current() polygon if
        // there was a move to the next polygon. The size of the next polygon
        // is set to remainingSize
        bool handleSkip( const QPointF& point,
            Point*& dataPtr, int& currSize, int remainingSize )
        {
            bool isNaN = ( qIsNaN( point.x() ) || qIsNaN( point.y() ) );
            if ( isNaN && !m_polygons.back().empty())
            {
                m_polygons.back().resize(currSize);
                currSize = 0;
                m_polygons.push_back( Polygon() );
                m_polygons.back().resize( remainingSize );
                dataPtr = m_polygons.back().data();
            }
            return isNaN;
        }

        Polygon& current() { return m_polygons.back(); }

        const Polygon& current() const { return m_polygons.back(); }

    private:
        QVector<Polygon>& m_polygons;
    };
}

// note that last is not inclusive
template <class MapperOutput, class Point>
static int handleSkippedPointsInRange( MapperOutput& mapperOutput,
    const QwtSeriesData<Point> *series, int first, int last )
{
    if ( MapperOutput::isNone )
        return first;

    while ( first < last && mapperOutput.handleSkip( series->sample( first ) ) )
    {
        first++;
    }
    return first;
}

// note that last is not inclusive
template <class Point, class MapperOutput>
static int handleSkippedPointsInRange( MapperOutput& mapperOutput,
    const Point *series, int first, int last )
{
    if ( MapperOutput::isNone )
        return first;

    while ( first < last && mapperOutput.handleSkip( series[ first ] ) )
    {
        first++;
    }
    return first;
}

template <class MapperOutput, class PolygonQuadrupel>
static void qwtMapPointsQuad( MapperOutput& output,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to )
{
    from = handleSkippedPointsInRange( output, series, from, to + 1 );
    if ( from > to )
        return;

    const QPointF sample0 = series->sample( from );

    PolygonQuadrupel q;
    q.start( qwtRoundValue( xMap.transform( sample0.x() ) ),
        qwtRoundValue( yMap.transform( sample0.y() ) ) );

    for ( int i = from; i <= to; i++ )
    {
        const QPointF sample = series->sample( i );

        if ( output.handleSkip( sample ))
            continue;

        const int x = qwtRoundValue( xMap.transform( sample.x() ) );
        const int y = qwtRoundValue( yMap.transform( sample.y() ) );

        if ( !q.append( x, y ) )
        {
            q.flush( output );
            q.start( x, y );
        }
    }
    q.flush( output );
}

template <class MapperOutput, class PolygonQuadrupel>
static void qwtMapPointsQuad( MapperOutput& output )
{
    typedef typename MapperOutput::PointType Point;
    typedef typename MapperOutput::PolygonType Polygon;

    const int numPoints = output.current().size();

    if ( MapperOutput::isNone && numPoints < 3 )
    {
        return;
    }

    Polygon sourcePolygon;
    sourcePolygon.swap( output.current() );
    const Point *points = sourcePolygon.constData();

    PolygonQuadrupel q;

    int i = handleSkippedPointsInRange( output, points, 0, numPoints );
    if ( i >= numPoints )
    {
        return;
    }

    q.start( points[i].x(), points[i].y() );

    for ( ; i < numPoints; i++ )
    {
        if ( output.handleSkip( points[i] ) )
            continue;

        const int x = points[i].x();
        const int y = points[i].y();

        if ( !q.append( x, y ) )
        {
            q.flush( output );
            q.start( x, y );
        }
    }
    q.flush( output );
}


template <class MapperOutput>
static void qwtMapPointsQuad( MapperOutput& output,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to )
{
    typedef typename MapperOutput::PointType Point;

    if ( from > to )
        return;

    /*
        probing some values, to decide if it is better
        to start with x or y coordinates
     */
    const Qt::Orientation orientation = qwtProbeOrientation( series, from, to );

    if ( orientation == Qt::Horizontal )
    {
        qwtMapPointsQuad< MapperOutput,
            QwtPolygonQuadrupelY<MapperOutput, Point>
            >( output, xMap, yMap, series, from, to );

        qwtMapPointsQuad< MapperOutput,
            QwtPolygonQuadrupelX<MapperOutput, Point> >( output );
    }
    else
    {
        qwtMapPointsQuad< MapperOutput,
            QwtPolygonQuadrupelX<MapperOutput, Point>
            >( output, xMap, yMap, series, from, to );

        qwtMapPointsQuad< MapperOutput,
            QwtPolygonQuadrupelY<MapperOutput, Point> >( output );
    }
}

// Helper class to work around the 5 parameters
// limitation of QtConcurrent::run()
class QwtDotsCommand
{
public:
    const QwtSeriesData<QPointF> *series;
    int from;
    int to;
    QRgb rgb;
};

static void qwtRenderDots(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtDotsCommand &command, const QPoint &pos, QImage *image )
{
    const QRgb rgb = command.rgb;
    QRgb *bits = reinterpret_cast<QRgb *>( image->bits() );

    const int w = image->width();
    const int h = image->height();

    const int x0 = pos.x();
    const int y0 = pos.y();

    for ( int i = command.from; i <= command.to; i++ )
    {
        const QPointF sample = command.series->sample( i );

        const int x = static_cast<int>( xMap.transform( sample.x() ) + 0.5 ) - x0;
        const int y = static_cast<int>( yMap.transform( sample.y() ) + 0.5 ) - y0;

        if ( x >= 0 && x < w && y >= 0 && y < h )
            bits[ y * w + x ] = rgb;
    }
}

// some functors, so that the compile can inline
struct QwtRoundI
{
    inline int operator()( double value ) const
    {
        return qwtRoundValue( value );
    }
};

struct QwtRoundF
{
    inline double operator()( double value ) const
    {
        return qwtRoundValueF( value );
    }
};

struct QwtNoRoundF
{
    inline double operator()( double value ) const
    {
        return value;
    }
};

// mapping points without any filtering - beside checking
// the bounding rectangle

template<class MapperOutput, class Round>
static inline void qwtToPoints(
    MapperOutput& output,
    const QRectF &boundingRect,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series,
    int from, int to, Round round )
{
    typedef typename MapperOutput::PointType Point;
    output.current().resize( to - from + 1 );
    Point *points = output.current().data();

    int numPoints = 0;

    if ( boundingRect.isValid() )
    {
        // iterating over all values
        // filtering out all points outside of
        // the bounding rectangle

        for ( int i = from; i <= to; i++ )
        {
            const QPointF sample = series->sample( i );

            if ( output.handleSkip( sample, points, numPoints, to - i + 1 ) )
                continue;

            const double x = xMap.transform( sample.x() );
            const double y = yMap.transform( sample.y() );

            if ( boundingRect.contains( x, y ) )
            {
                points[ numPoints ].rx() = round( x );
                points[ numPoints ].ry() = round( y );

                numPoints++;
            }
        }

        output.current().resize( numPoints );
    }
    else
    {
        // simply iterating over all values
        // without any filtering

        for ( int i = from; i <= to; i++ )
        {
            const QPointF sample = series->sample( i );

            if ( output.handleSkip( sample, points, numPoints, to - i + 1 ) )
            {
                continue;
            }

            const double x = xMap.transform( sample.x() );
            const double y = yMap.transform( sample.y() );

            points[ numPoints ].rx() = round( x );
            points[ numPoints ].ry() = round( y );

            numPoints++;
        }
        output.current().resize( numPoints );
    }
}

// Mapping points with filtering out consecutive
// points mapped to the same position

template<class MapperOutput, class Round>
static inline void qwtToPolylineFiltered(
    MapperOutput& output,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series,
    int from, int to, Round round )
{
    // in curves with many points consecutive points
    // are often mapped to the same position. As this might
    // result in empty lines ( or symbols hidden by others )
    // we try to filter them out
    from = handleSkippedPointsInRange( output, series, from, to );
    if ( from > to )
        return;

    typedef typename MapperOutput::PointType Point;
    output.current().resize( to - from + 1 );
    Point *points = output.current().data();

    const QPointF sample0 = series->sample( from );

    points[0].rx() = round( xMap.transform( sample0.x() ) );
    points[0].ry() = round( yMap.transform( sample0.y() ) );

    int numPoints = 1;
    for ( int i = from + 1; i <= to; i++ )
    {
        const QPointF sample = series->sample( i );

        if ( output.handleSkip( sample, points, numPoints, to - i + 1 ) )
        {
            continue;
        }

        const Point p( round( xMap.transform( sample.x() ) ),
            round( yMap.transform( sample.y() ) ) );

        if ( points[numPoints - 1] != p )
            points[numPoints++] = p;
    }

    output.current().resize( numPoints );
}

template<class MapperOutput>
static inline void qwtToPointsFiltered(
    MapperOutput& output,
    const QRectF &boundingRect,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to )
{
    // F.e. in scatter plots ( no connecting lines ) we
    // can sort out all duplicates ( not only consecutive points )

    typedef typename MapperOutput::PointType Point;
    output.current().resize( to - from + 1 );
    Point *points = output.current().data();

    QwtPixelMatrix pixelMatrix( boundingRect.toAlignedRect() );

    int numPoints = 0;
    for ( int i = from; i <= to; i++ )
    {
        const QPointF sample = series->sample( i );

        if ( output.handleSkip( sample, points, numPoints, to - i + 1 ) )
            continue;

        const int x = qwtRoundValue( xMap.transform( sample.x() ) );
        const int y = qwtRoundValue( yMap.transform( sample.y() ) );

        if ( pixelMatrix.testAndSetPixel( x, y, true ) == false )
        {
            points[ numPoints ].rx() = x;
            points[ numPoints ].ry() = y;

            numPoints++;
        }
    }

    output.current().resize( numPoints );
}

template <class PointSkipStrategy >
static QPolygonF qwtToPointsFilteredF(
    const QRectF &boundingRect,
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to )
{
    return qwtToPointsFiltered<QPolygonF, QPointF, PointSkipStrategy>(
        boundingRect, xMap, yMap, series, from, to );
}

class QwtPointMapper::PrivateData
{
public:
    PrivateData():
        boundingRect( qwtInvalidRect )
    {
    }

    QRectF boundingRect;
    QwtPointMapper::TransformationFlags flags;
};

//! Constructor
QwtPointMapper::QwtPointMapper()
{
    d_data = new PrivateData();
}

//! Destructor
QwtPointMapper::~QwtPointMapper()
{
    delete d_data;
}

/*!
  Set the flags affecting the transformation process

  \param flags Flags
  \sa flags(), setFlag()
 */
void QwtPointMapper::setFlags( TransformationFlags flags )
{
    d_data->flags = flags;
}

/*!
  \return Flags affecting the transformation process
  \sa setFlags(), setFlag()
 */
QwtPointMapper::TransformationFlags QwtPointMapper::flags() const
{
    return d_data->flags;
}

/*!
  Modify a flag affecting the transformation process

  \param flag Flag type
  \param on Value

  \sa flag(), setFlags()
 */
void QwtPointMapper::setFlag( TransformationFlag flag, bool on )
{
    if ( on )
        d_data->flags |= flag;
    else
        d_data->flags &= ~flag;
}

/*!
  \return True, when the flag is set
  \param flag Flag type
  \sa setFlag(), setFlags()
 */
bool QwtPointMapper::testFlag( TransformationFlag flag ) const
{
    return d_data->flags & flag;
}

/*!
  Set a bounding rectangle for the point mapping algorithm

  A valid bounding rectangle can be used for optimizations

  \param rect Bounding rectangle
  \sa boundingRect()
 */
void QwtPointMapper::setBoundingRect( const QRectF &rect )
{
    d_data->boundingRect = rect;
}

/*!
  \return Bounding rectangle
  \sa setBoundingRect()
 */
QRectF QwtPointMapper::boundingRect() const
{
    return d_data->boundingRect;
}

/*!
  \brief Translate a series of points into a QPolygonF

  When the WeedOutPoints flag is enabled consecutive points,
  that are mapped to the same position will be one point.

  When RoundPoints is set all points are rounded to integers
  but returned as PolygonF - what only makes sense
  when the further processing of the values need a QPolygonF.

  When RoundPoints & WeedOutIntermediatePoints is enabled an even more
  aggressive weeding algorithm is enabled.

  When the HandleNaNAsDiscontinuity flag is enabled,
  points which have one or more coordinates as NaN will be ignored.

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted

  \return Translated polygon
*/
QPolygonF QwtPointMapper::toPolygonF(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to ) const
{
    QPolygonF polyline;

    if ( d_data->flags & RoundPoints )
    {
        if ( d_data->flags & WeedOutIntermediatePoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( polyline );
                qwtMapPointsQuad(wrapper, xMap, yMap, series, from, to );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( polyline );
                qwtMapPointsQuad(wrapper, xMap, yMap, series, from, to );
            }
        }
        else if ( d_data->flags & WeedOutPoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( polyline );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( polyline );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
        }
        else
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( polyline );
                qwtToPoints( wrapper, qwtInvalidRect,
                     xMap, yMap, series, from, to, QwtRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( polyline );
                qwtToPoints( wrapper, qwtInvalidRect,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
        }
    }
    else
    {
        if ( d_data->flags & WeedOutPoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( polyline );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( polyline );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
        }
        else
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( polyline );
                qwtToPoints( wrapper, qwtInvalidRect,
                     xMap, yMap, series, from, to, QwtNoRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( polyline );
                qwtToPoints( wrapper, qwtInvalidRect,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
        }
    }

    return polyline;
}

/*!
  \brief Translate a series of points into a QPolygon

  When the WeedOutPoints flag is enabled consecutive points,
  that are mapped to the same position will be one point.

  When the HandleNaNAsDiscontinuity flag is enabled,
  points which have one or more coordinates as NaN will be ignored.

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted

  \return Translated polygon
*/
QPolygon QwtPointMapper::toPolygon(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to ) const
{
    QPolygon polyline;

    if ( d_data->flags & WeedOutIntermediatePoints )
    {
        // TODO WeedOutIntermediatePointsY ...
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputOmitNaN< QPolygon, QPoint > wrapper( polyline );
            qwtMapPointsQuad( wrapper, xMap, yMap, series, from, to );
        }
        else
        {
            QwtMapperOutputDontSkip< QPolygon, QPoint > wrapper( polyline );
            qwtMapPointsQuad( wrapper, xMap, yMap, series, from, to );
        }
    }
    else if ( d_data->flags & WeedOutPoints )
    {
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputOmitNaN< QPolygon, QPoint > wrapper( polyline );
            qwtToPolylineFiltered(wrapper,
                xMap, yMap, series, from, to, QwtRoundI() );
        }
        else
        {
            QwtMapperOutputDontSkip< QPolygon, QPoint > wrapper( polyline );
            qwtToPolylineFiltered(wrapper,
                xMap, yMap, series, from, to, QwtRoundI() );
        }
    }
    else
    {
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputOmitNaN< QPolygon, QPoint > wrapper( polyline );
            qwtToPoints(wrapper, qwtInvalidRect, xMap, yMap, series, from, to, QwtRoundI() );
        }
        else
        {
            QwtMapperOutputDontSkip< QPolygon, QPoint > wrapper( polyline );
            qwtToPoints(wrapper, qwtInvalidRect, xMap, yMap, series, from, to, QwtRoundI() );
        }
    }

    return polyline;
}

/*!
  \brief Translate a series of points into a one or more QPolygon

  When the WeedOutPoints flag is enabled consecutive points,
  that are mapped to the same position will be one point.

  When the HandleNaNAsDiscontinuity flag is enabled,
  points which have one or more coordinates as NaN will result in
  creation of separate polygon for each set of contiguous non-NaN
  points.

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted

  \return Translated polygons
*/
QVector<QPolygonF> QwtPointMapper::toPolygonsF(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to ) const
{
    QVector<QPolygonF> polylines;

    if ( d_data->flags & RoundPoints )
    {
        if ( d_data->flags & WeedOutIntermediatePoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputMultipleSplitNaNToPolygons< QPolygonF, QPointF > wrapper( polylines );
                qwtMapPointsQuad(wrapper, xMap, yMap, series, from, to );
            }
            else
            {
                QwtMapperOutputMultipleDontSkip< QPolygonF, QPointF > wrapper( polylines );
                qwtMapPointsQuad(wrapper, xMap, yMap, series, from, to );
            }
        }
        else if ( d_data->flags & WeedOutPoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputMultipleSplitNaNToPolygons< QPolygonF, QPointF > wrapper( polylines );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
            else
            {
                QwtMapperOutputMultipleDontSkip< QPolygonF, QPointF > wrapper( polylines );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
        }
        else
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputMultipleSplitNaNToPolygons< QPolygonF, QPointF > wrapper( polylines );
                qwtToPoints( wrapper, qwtInvalidRect,
                     xMap, yMap, series, from, to, QwtRoundF() );
            }
            else
            {
                QwtMapperOutputMultipleDontSkip< QPolygonF, QPointF > wrapper( polylines );
                qwtToPoints( wrapper, qwtInvalidRect,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
        }
    }
    else
    {
        if ( d_data->flags & WeedOutPoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputMultipleSplitNaNToPolygons< QPolygonF, QPointF > wrapper( polylines );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
            else
            {
                QwtMapperOutputMultipleDontSkip< QPolygonF, QPointF > wrapper( polylines );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
        }
        else
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputMultipleSplitNaNToPolygons< QPolygonF, QPointF > wrapper( polylines );
                qwtToPoints( wrapper, qwtInvalidRect,
                     xMap, yMap, series, from, to, QwtNoRoundF() );
            }
            else
            {
                QwtMapperOutputMultipleDontSkip< QPolygonF, QPointF > wrapper( polylines );
                qwtToPoints( wrapper, qwtInvalidRect,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
        }
    }

    return polylines;
}

/*!
  \brief Translate a series of points into a one or more QPolygon

  When the WeedOutPoints flag is enabled consecutive points,
  that are mapped to the same position will be one point.

  When the HandleNaNAsDiscontinuity flag is enabled,
  points which have one or more coordinates as NaN will result in
  creation of separate polygon for each set of contiguous non-NaN
  points.

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted

  \return Translated polygons
*/
QVector<QPolygon> QwtPointMapper::toPolygons(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to ) const
{
    QVector<QPolygon> polylines;

    if ( d_data->flags & WeedOutIntermediatePoints )
    {
        // TODO WeedOutIntermediatePointsY ...
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputMultipleSplitNaNToPolygons< QPolygon, QPoint > wrapper( polylines );
            qwtMapPointsQuad( wrapper, xMap, yMap, series, from, to );
        }
        else
        {
            QwtMapperOutputMultipleDontSkip< QPolygon, QPoint > wrapper( polylines );
            qwtMapPointsQuad( wrapper, xMap, yMap, series, from, to );
        }
    }
    else if ( d_data->flags & WeedOutPoints )
    {
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputMultipleSplitNaNToPolygons< QPolygon, QPoint > wrapper( polylines );
            qwtToPolylineFiltered(wrapper,
                xMap, yMap, series, from, to, QwtRoundI() );
        }
        else
        {
            QwtMapperOutputMultipleDontSkip< QPolygon, QPoint > wrapper( polylines );
            qwtToPolylineFiltered(wrapper,
                xMap, yMap, series, from, to, QwtRoundI() );
        }
    }
    else
    {
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputMultipleSplitNaNToPolygons< QPolygon, QPoint > wrapper( polylines );
            qwtToPoints(wrapper, qwtInvalidRect, xMap, yMap, series, from, to, QwtRoundI() );
        }
        else
        {
            QwtMapperOutputMultipleDontSkip< QPolygon, QPoint > wrapper( polylines );
            qwtToPoints(wrapper, qwtInvalidRect, xMap, yMap, series, from, to, QwtRoundI() );
        }
    }

    return polylines;
}

/*!
  \brief Translate a series into a QPolygonF

  - WeedOutPoints & RoundPoints & boundingRect().isValid()
    All points that are mapped to the same position
    will be one point. Points outside of the bounding
    rectangle are ignored.

  - WeedOutPoints & RoundPoints & !boundingRect().isValid()
    All consecutive points that are mapped to the same position
    will one point

  - WeedOutPoints & !RoundPoints
    All consecutive points that are mapped to the same position
    will one point

  - !WeedOutPoints & boundingRect().isValid()
    Points outside of the bounding rectangle are ignored.

  When RoundPoints is set all points are rounded to integers
  but returned as PolygonF - what only makes sense
  when the further processing of the values need a QPolygonF.

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted

  \return Translated polygon
*/
QPolygonF QwtPointMapper::toPointsF(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to ) const
{
    QPolygonF points;

    if ( d_data->flags & WeedOutPoints )
    {
        if ( d_data->flags & RoundPoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                if ( d_data->boundingRect.isValid() )
                {
                    QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( points );
                    qwtToPointsFiltered( wrapper, d_data->boundingRect,
                        xMap, yMap, series, from, to );
                }
                else
                {
                    // without a bounding rectangle all we can
                    // do is to filter out duplicates of
                    // consecutive points
                    QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( points );
                    qwtToPolylineFiltered(wrapper,
                        xMap, yMap, series, from, to, QwtRoundF() );
                }
            }
            else
            {
                if ( d_data->boundingRect.isValid() )
                {
                    QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( points );
                    qwtToPointsFiltered( wrapper, d_data->boundingRect,
                        xMap, yMap, series, from, to );
                }
                else
                {
                    // without a bounding rectangle all we can
                    // do is to filter out duplicates of
                    // consecutive points
                    QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( points );
                    qwtToPolylineFiltered( wrapper,
                        xMap, yMap, series, from, to, QwtRoundF() );
                }
            }
        }
        else
        {
            // when rounding is not allowed we can't use
            // qwtToPointsFilteredF
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( points );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( points );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
        }
    }
    else
    {
        if ( d_data->flags & RoundPoints )
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( points );
                qwtToPoints( wrapper, d_data->boundingRect,
                     xMap, yMap, series, from, to, QwtRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( points );
                qwtToPoints( wrapper, d_data->boundingRect,
                    xMap, yMap, series, from, to, QwtRoundF() );
            }
        }
        else
        {
            if ( d_data->flags & HandleNaNAsDiscontinuity )
            {
                QwtMapperOutputOmitNaN< QPolygonF, QPointF > wrapper( points );
                qwtToPoints( wrapper, d_data->boundingRect,
                     xMap, yMap, series, from, to, QwtNoRoundF() );
            }
            else
            {
                QwtMapperOutputDontSkip< QPolygonF, QPointF > wrapper( points );
                qwtToPoints( wrapper, d_data->boundingRect,
                    xMap, yMap, series, from, to, QwtNoRoundF() );
            }
        }
    }

    return points;
}

/*!
  \brief Translate a series of points into a QPolygon

  - WeedOutPoints & boundingRect().isValid()
    All points that are mapped to the same position
    will be one point. Points outside of the bounding
    rectangle are ignored.

  - WeedOutPoints & !boundingRect().isValid()
    All consecutive points that are mapped to the same position
    will one point

  - !WeedOutPoints & boundingRect().isValid()
    Points outside of the bounding rectangle are ignored.

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted

  \return Translated polygon
*/
QPolygon QwtPointMapper::toPoints(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to ) const
{
    QPolygon points;

    if ( d_data->flags & WeedOutPoints )
    {
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            if ( d_data->boundingRect.isValid() )
            {
                QwtMapperOutputOmitNaN< QPolygon, QPoint > wrapper( points );
                qwtToPointsFiltered( wrapper, d_data->boundingRect,
                    xMap, yMap, series, from, to );
            }
            else
            {
                // when we don't have the bounding rectangle all
                // we can do is to filter out consecutive duplicates

                QwtMapperOutputOmitNaN< QPolygon, QPoint > wrapper( points );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtRoundI() );
            }
        }
        else
        {
            if ( d_data->boundingRect.isValid() )
            {
                QwtMapperOutputDontSkip< QPolygon, QPoint > wrapper( points );
                qwtToPointsFiltered( wrapper, d_data->boundingRect,
                    xMap, yMap, series, from, to );
            }
            else
            {
                // when we don't have the bounding rectangle all
                // we can do is to filter out consecutive duplicates

                QwtMapperOutputDontSkip< QPolygon, QPoint > wrapper( points );
                qwtToPolylineFiltered(wrapper,
                    xMap, yMap, series, from, to, QwtRoundI() );
            }
        }
    }
    else
    {
        if ( d_data->flags & HandleNaNAsDiscontinuity )
        {
            QwtMapperOutputOmitNaN< QPolygon, QPoint > wrapper( points );
            qwtToPoints(wrapper, d_data->boundingRect,
                xMap, yMap, series, from, to, QwtRoundI() );
        }
        else
        {
            QwtMapperOutputDontSkip< QPolygon, QPoint > wrapper( points );
            qwtToPoints(wrapper, d_data->boundingRect,
                xMap, yMap, series, from, to, QwtRoundI() );
        }
    }

    return points;
}


/*!
  \brief Translate a series into a QImage

  \param xMap x map
  \param yMap y map
  \param series Series of points to be mapped
  \param from Index of the first point to be painted
  \param to Index of the last point to be painted
  \param pen Pen used for drawing a point
             of the image, where a point is mapped to
  \param antialiased True, when the dots should be displayed
                     antialiased
  \param numThreads Number of threads to be used for rendering.
                   If numThreads is set to 0, the system specific
                   ideal thread count is used.

  \return Image displaying the series
*/
QImage QwtPointMapper::toImage(
    const QwtScaleMap &xMap, const QwtScaleMap &yMap,
    const QwtSeriesData<QPointF> *series, int from, int to,
    const QPen &pen, bool antialiased, uint numThreads ) const
{
    Q_UNUSED( antialiased )

#if QWT_USE_THREADS
    if ( numThreads == 0 )
        numThreads = QThread::idealThreadCount();

    if ( numThreads <= 0 )
        numThreads = 1;
#else
    Q_UNUSED( numThreads )
#endif

    // a very special optimization for scatter plots
    // where every sample is mapped to one pixel only.

    const QRect rect = d_data->boundingRect.toAlignedRect();

    QImage image( rect.size(), QImage::Format_ARGB32 );
    image.fill( Qt::transparent );

    if ( pen.width() <= 1 && pen.color().alpha() == 255 )
    {
        QwtDotsCommand command;
        command.series = series;
        command.rgb = pen.color().rgba();

#if QWT_USE_THREADS
        const int numPoints = ( to - from + 1 ) / numThreads;

        QList< QFuture<void> > futures;
        for ( uint i = 0; i < numThreads; i++ )
        {
            const QPoint pos = rect.topLeft();

            const int index0 = from + i * numPoints;
            if ( i == numThreads - 1 )
            {
                command.from = index0;
                command.to = to;

                qwtRenderDots( xMap, yMap, command, pos, &image );
            }
            else
            {
                command.from = index0;
                command.to = index0 + numPoints - 1;

                futures += QtConcurrent::run( &qwtRenderDots,
                    xMap, yMap, command, pos, &image );
            }
        }
        for ( int i = 0; i < futures.size(); i++ )
            futures[i].waitForFinished();
#else
        command.from = from;
        command.to = to;

        qwtRenderDots( xMap, yMap, command, rect.topLeft(), &image );
#endif
    }
    else
    {
        // fallback implementation: to be replaced later by
        // setting the pixels of the image like above, TODO ...

        QPainter painter( &image );
        painter.setPen( pen );
        painter.setRenderHint( QPainter::Antialiasing, antialiased );

        const int chunkSize = 1000;
        for ( int i = from; i <= to; i += chunkSize )
        {
            const int indexTo = qMin( i + chunkSize - 1, to );
            const QPolygon points = toPoints(
                xMap, yMap, series, i, indexTo );

            painter.drawPoints( points );
        }
    }

    return image;
}
