/* -*- mode: C++ ; c-file-style: "stroustrup" -*- *****************************
 * Qwt Widget Library
 * Copyright (C) 2019  Povilas Kanapickas <povilas@radix.lt>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the Qwt License, Version 1.0
 *****************************************************************************/

#include "qwt_global.h"
#include "qwt_bounding_rect_cache.h"
#include <algorithm>

static const unsigned kLevelFactor = 8;

class QwtHierarchicalRectStore
{
    /* Stores a hierarchy of bounding rectangles to allow querying of total bounding
       rectangle of all items in a range between i-th rectangle and j-th rectangle.

       The 0-th level stores all rectangles as-is.

       The 1-th level stores one bounding rectangle for kLevelFactor rectangles in the first level.

       The 2-th level stores one bounding rectangle for kLevelFactor rectangles in the second level.

       And so on.

       The algorithmic complexity to query this data structure is O(log(n)) instead of O(n)
       as is in the trivial scanning implementation.
    */
public:
    // The first min(size, levelSize(0)) rectangles of the 0-th layer are guaranteed to be
    // left intact
    void resize( size_t size )
    {
        d_levelSizes.clear();

        d_levelSizes.push_back( size );
        size_t totalSize = size;
        size /= kLevelFactor;

        while ( size > 0 )
        {
            d_levelSizes.push_back( size );
            totalSize += size;
            size /= kLevelFactor;
        }

        d_rects.resize( totalSize );
    }

    size_t levelCount() const
    {
        return d_levelSizes.size();
    }

    size_t levelSize( size_t level ) const
    {
        if ( level >= (size_t)d_levelSizes.size() )
            return 0;
        return d_levelSizes[ level ];
    }

    size_t levelOffset( size_t level ) const
    {
        size_t offset = 0;
        for ( size_t i = 0; i < level; ++i )
        {
            offset += levelSize( i );
        }
        return offset;
    }

    QRectF* getLevel( size_t level )
    {
        return d_rects.data() + levelOffset( level );
    }

    const QRectF* getLevel( size_t level ) const
    {
        return d_rects.data() + levelOffset( level );
    }

private:
    QVector<size_t> d_levelSizes;
    QVector<QRectF> d_rects;
};

class QwtBoundingRectCache::PrivateData
{
public:
    PrivateData() : orientation( Qt::Horizontal ) {}
    QwtHierarchicalRectStore rectStore;
    Qt::Orientation orientation;
};

QwtBoundingRectCache::QwtBoundingRectCache() :
    d_data( new PrivateData )
{
}

QwtBoundingRectCache::~QwtBoundingRectCache()
{
    delete d_data;
}

static inline bool rectLeftLess( const QRectF& lhs, const QRectF& rhs )
{
    return lhs.left() < rhs.left();
}

static inline bool doubleRectLeftLess( const double& lhs, const QRectF& rhs )
{
    return lhs < rhs.left();
}

static inline bool rectLeftDoubleLess( const QRectF& lhs, const double& rhs )
{
    return lhs.left() < rhs;
}

static inline QRectF rotateRect( const QRectF& rect )
{
    return QRectF( rect.top(), rect.left(), rect.height(), rect.width() );
}

static inline bool qwtIsNaN( const QRectF& rect )
{
    return qIsNaN( rect.x() ) || qIsNaN( rect.y() ) ||
           qIsNaN( rect.width() ) || qIsNaN( rect.height() );
}

void QwtBoundingRectCache::build( QwtAbstractBoundingRectProducer* rectProducer,
                                  Qt::Orientation orientation )
{
    size_t size = rectProducer->dataSize();
    d_data->rectStore.resize( size );

    // fill first layer
    QRectF* level = d_data->rectStore.getLevel( 0 );

    size_t levelSize = 0;
    if ( orientation == Qt::Vertical )
    {
        for ( size_t i = 0; i < size ; ++i )
        {
            const QRectF& rect = rectProducer->sampleRect( i );
            if ( qwtIsNaN( rect ) )
                continue;
            level[ levelSize++ ] = rotateRect( rectProducer->sampleRect( i ) );
        }
    }
    else
    {
        for ( size_t i = 0; i < size ; ++i )
        {
            const QRectF& rect = rectProducer->sampleRect( i );
            if ( qwtIsNaN( rect ) )
                continue;
            level[ levelSize++ ] = rect;
        }
    }

    d_data->rectStore.resize( levelSize );

    std::sort( level, level + levelSize, rectLeftLess );

    // fill subsequent layers
    for ( size_t iLevel = 1; iLevel < d_data->rectStore.levelCount(); ++iLevel )
    {
        levelSize = d_data->rectStore.levelSize( iLevel );
        level = d_data->rectStore.getLevel( iLevel );
        QRectF* prevLevel = d_data->rectStore.getLevel( iLevel - 1 );

        for ( size_t i = 0; i < levelSize ; ++i )
        {
            size_t prevLevelIndex = i * kLevelFactor;
            QRectF rect = prevLevel[ prevLevelIndex ];
            for ( size_t iRect = 1; iRect < kLevelFactor; ++iRect )
            {
                rect = qwtMergeBoundingRect( rect, prevLevel[ prevLevelIndex + iRect ] );
            }
            level[ i ] = rect;
        }
    }
}

static inline size_t ceilBoundary( size_t from )
{
    size_t boundary = (from / kLevelFactor) * kLevelFactor;
    if ( boundary != from )
        boundary += kLevelFactor;
    return boundary;
}

static inline size_t floorBoundary( size_t to )
{
    return (to / kLevelFactor) * kLevelFactor;
}

QRectF QwtBoundingRectCache::convertRectToOrientation( const QRectF& rect ) const
{
    if ( d_data->orientation == Qt::Vertical )
    {
        return rotateRect( rect );
    }
    return rect;
}

QRectF QwtBoundingRectCache::boundingRect( double fromValue, double toValue ) const
{
    if ( toValue <= fromValue )
        return QRectF( 0, 0, -1, -1 );

    // first, get the indexes to the bounding rects corresponding the nearest values within
    // [fromValue, toValue] interval.
    size_t firstLevelSize = d_data->rectStore.levelSize( 0 );
    const QRectF* firstLayer = d_data->rectStore.getLevel( 0 );

    const QRectF* fromRect = std::lower_bound( firstLayer, firstLayer + firstLevelSize, fromValue,
                                               rectLeftDoubleLess );
    // note that toRect is not inclusive
    const QRectF* toRect = std::upper_bound( firstLayer, firstLayer + firstLevelSize, toValue,
                                             doubleRectLeftLess );

    size_t from = fromRect - firstLayer;
    size_t to = toRect - firstLayer;

    if ( from >= to || from == firstLevelSize || to == 0 )
    {
        return QRectF( 0, 0, -1, -1 );
    }

    // rectangle indexes are known, now load values from cache
    QRectF res = *fromRect;

    for ( size_t iLevel = 0; iLevel < d_data->rectStore.levelCount(); ++iLevel )
    {
        const QRectF* level = d_data->rectStore.getLevel( iLevel );
        size_t fromBoundary = ceilBoundary( from );
        size_t toBoundary = floorBoundary( to );

        if ( fromBoundary >= toBoundary )
        {
            // this level was the last
            for ( ; from <= to; from++ )
            {
                res = qwtMergeBoundingRect( res, level[ from ] );
            }
            return convertRectToOrientation( res );
        }

        // at least one more level will be examined
        for ( size_t i = from; i < fromBoundary; ++i )
        {
            res = qwtMergeBoundingRect( res, level[ i ] );
        }
        for ( size_t i = toBoundary; i < to; ++i )
        {
            res = qwtMergeBoundingRect( res, level[ i - 1] );
        }

        from = fromBoundary / kLevelFactor;
        to = toBoundary / kLevelFactor;
    }

    // should never happen
    return convertRectToOrientation( res );
}
