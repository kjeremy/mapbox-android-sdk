package com.mapbox.mapboxsdk.views.util;

import android.graphics.Matrix;
import android.graphics.Point;
import android.graphics.PointF;
import android.graphics.Rect;

import com.mapbox.mapboxsdk.api.ILatLng;
import com.mapbox.mapboxsdk.constants.GeoConstants;
import com.mapbox.mapboxsdk.geometry.BoundingBox;
import com.mapbox.mapboxsdk.geometry.LatLng;
import com.mapbox.mapboxsdk.tileprovider.constants.TileLayerConstants;
import com.mapbox.mapboxsdk.util.GeometryMath;
import com.mapbox.mapboxsdk.views.MapView;

    /**
     * A Projection serves to translate between the coordinate system of x/y on-screen pixel coordinates
     * and that of latitude/longitude points on the surface of the earth. You obtain a Projection from
     * MapView.getProjection(). You should not hold on to this object for more than one draw, since the
     * projection of the map could change. <br>
     * <br>
     * <b>Note:</b> This class will "wrap" all pixel and lat/long values that overflow their bounds
     * (rather than clamping to their bounds).
     *
     * @author Marc Kurtz
     * @author Nicolas Gramlich
     * @author Manuel Stahl
     */
    public class Projection implements GeoConstants {
        private final int mMapViewWidth;
        private final int mMapViewHeight;

        // The offsets will take us from the MapView's current coordinate system
        // to a 0,0 coordinate system
        protected final int mOffsetX;
        protected final int mOffsetY;

        protected final float mMultiTouchScale;
        private final Matrix mRotateAndScaleMatrix = new Matrix();
        private final Matrix mUnrotateAndScaleMatrix = new Matrix();
        private final float[] mRotateScalePoints = new float[2];
        private final BoundingBox mBoundingBoxProjection;
        private final float mZoomLevelProjection;
        private final Rect mScreenRectProjection;
        private final Rect mIntrinsicScreenRectProjection;
        private final float mMapOrientation;

        protected static int mTileSize = 256;

        public Projection(MapView mapView) {
            mZoomLevelProjection = mapView.getZoomLevel(false);
            mIntrinsicScreenRectProjection = mapView.getIntrinsicScreenRect(null);

            if (mapView.getMapOrientation() % 180 != 0) {
                // Since the canvas is shifted by getWidth/2, we can just return our
                // natural scrollX/Y
                // value since that is the same as the shifted center.
                PointF scrollPoint = mapView.getScrollPoint();
                mScreenRectProjection = GeometryMath.getBoundingBoxForRotatedRectangle(mIntrinsicScreenRectProjection,
                        scrollPoint.x, scrollPoint.y, this.getMapOrientation(), null);
            } else {
                mScreenRectProjection = mIntrinsicScreenRectProjection;
            }

            mMapOrientation = mapView.getMapOrientation();
            mMapViewWidth = mapView.getWidth();
            mMapViewHeight = mapView.getHeight();
            mOffsetX = -mapView.getScrollX();
            mOffsetY = -mapView.getScrollY();

            //mRotateAndScaleMatrix.set(mapView.mRotateScaleMatrix);
            mRotateAndScaleMatrix.setScale(mapView.getScale(), mapView.getScale(),
                    mapView.getScalePoint().x, mapView.getScalePoint().y);
            mRotateAndScaleMatrix.setRotate(mapView.getRotation(),
                    mMapViewWidth / 2, mMapViewHeight / 2  );

            mRotateAndScaleMatrix.invert(mUnrotateAndScaleMatrix);
            mMultiTouchScale = mapView.getScale();
            final ILatLng neGeoPoint = fromPixels(mMapViewWidth, 0);
            final ILatLng swGeoPoint = fromPixels(0, mMapViewHeight);
            mBoundingBoxProjection = new BoundingBox(neGeoPoint.getLatitude(),
                    neGeoPoint.getLongitude(), swGeoPoint.getLatitude(),
                    swGeoPoint.getLongitude());
        }

        public float getZoomLevel() {
            return mZoomLevelProjection;
        }

        public BoundingBox getBoundingBox() {
            return mBoundingBoxProjection;
        }

        public Rect getScreenRect() {
            return mScreenRectProjection;
        }

        public Rect getIntrinsicScreenRect() {
            return mIntrinsicScreenRectProjection;
        }

        public float getMapOrientation() {
            return mMapOrientation;
        }

        public ILatLng fromPixels(float x, float y) {
            return pixelXYToLatLong((double) x - mOffsetX, (double) y - mOffsetY, mZoomLevelProjection);
        }

        public PointF toPixels(final ILatLng in) {
            return toPixels(in, getZoomLevel(), null);
        }

        public PointF toPixels(final ILatLng in, PointF reuse) {
            return toPixels(in, getZoomLevel(), reuse);
        }

        public PointF toPixels(final ILatLng in, float zoomLevel, PointF reuse) {
            PointF out = reuse != null ? reuse : new PointF();
            out = latLongToPixelXY(in.getLatitude(), in.getLongitude(), zoomLevel, out);
            out = toPixelsFromMercator(out.x, out.y, out);
            out = adjustForDateLine(out.x, out.y, out);
            return out;
        }

        protected PointF adjustForDateLine(float x, float y, PointF reuse) {
            final PointF out = reuse != null ? reuse : new PointF();
            out.set(x, y);
            out.offset(-mMapViewWidth / 2, -mMapViewHeight / 2);
            final int mapSize = mapSize(getZoomLevel());
            final float absX = Math.abs(out.x);
            final float absY = Math.abs(out.y);
            if (absX > Math.abs(out.x - mapSize)) {
                out.x -= mapSize;
            }
            if (absX > Math.abs(out.x + mapSize)) {
                out.x += mapSize;
            }
            if (absY > Math.abs(out.y - mapSize)) {
                out.y -= mapSize;
            }
            if (absY > Math.abs(out.y + mapSize)) {
                out.y += mapSize;
            }
            out.offset(mMapViewWidth / 2, mMapViewHeight / 2);
            return out;
        }

        /**
         * A wrapper for {@link #toProjectedPixels(int, int, Point)}
         */
        public PointF toProjectedPixels(final ILatLng geoPoint, final PointF reuse) {
            return toProjectedPixels(geoPoint.getLatitude(), geoPoint.getLongitude(), reuse);
        }

        /**
         * Performs only the first computationally heavy part of the projection. Call
         * {@link #toPixelsFromProjected(Point, Point)} to get the final position.
         *
         * @param latitude  the latitute of the point
         * @param longitude the longitude of the point
         * @param reuse     just pass null if you do not have a Point to be 'recycled'.
         * @return intermediate value to be stored and passed to toMapPixelsTranslated.
         */
        public PointF toProjectedPixels(final double latitude, final double longitude, final PointF reuse) {
            return latLongToPixelXY(latitude, longitude, TileLayerConstants.MAXIMUM_ZOOMLEVEL, reuse);
        }

        /**
         * Performs the second computationally light part of the projection.
         *
         * @param in    the Point calculated by the {@link #toProjectedPixels(int, int, Point)}
         * @param reuse just pass null if you do not have a Point to be 'recycled'.
         * @return the Point containing the coordinates of the initial GeoPoint passed to the
         * {@link #toProjectedPixels(int, int, Point)}.
         */
        public PointF toPixelsFromProjected(final PointF in, final PointF reuse) {
            PointF out = reuse != null ? reuse : new PointF();
            final float zoomDifference = TileLayerConstants.MAXIMUM_ZOOMLEVEL - getZoomLevel();
            out.set(GeometryMath.rightShift(in.x, zoomDifference),
                    GeometryMath.rightShift(in.y, zoomDifference));
            out = toPixelsFromMercator(out.x, out.y, out);
            out = adjustForDateLine(out.x, out.y, out);
            return out;
        }

        public PointF toPixelsFromMercator(double x, double y, PointF reuse) {
            final PointF out = reuse != null ? reuse : new PointF();
            out.set((float)x, (float)y);
            out.offset(mOffsetX, mOffsetY);
            return out;
        }

        public PointF toMercatorPixels(int x, int y, PointF reuse) {
            final PointF out = reuse != null ? reuse : new PointF();
            out.set(x, y);
            out.offset(-mOffsetX, -mOffsetY);
            return out;
        }

        public float metersToEquatorPixels(final float meters) {
            return meters / (float) groundResolution(0, mZoomLevelProjection);
        }

        /**
         * Converts a distance in meters to one in (horizontal) pixels at the current zoomlevel and at
         * the current latitude at the center of the screen.
         *
         * @param meters the distance in meters
         * @return The number of pixels corresponding to the distance, if measured at the center of the
         * screen, at the current zoom level. The return value may only be approximate.
         */
        public float metersToPixels(final float meters) {
            return meters
                    / (float) groundResolution(getBoundingBox().getCenter().getLatitude(),
                    mZoomLevelProjection);
        }

        public ILatLng getNorthEast() {
            return fromPixels(mMapViewWidth, 0);
        }

        public ILatLng getSouthWest() {
            return fromPixels(0, mMapViewHeight);
        }

        /**
         * This will provide a Matrix that will revert the current map's scaling and rotation. This can
         * be useful when drawing to a fixed location on the screen.
         */
        public Matrix getInvertedScaleRotateCanvasMatrix() {
            return mUnrotateAndScaleMatrix;
        }

        /**
         * This will revert the current map's scaling and rotation for a point. This can be useful when
         * drawing to a fixed location on the screen.
         */
        public PointF unrotateAndScalePoint(int x, int y, PointF reuse) {
            if (reuse == null)
                reuse = new PointF();
            if (getMapOrientation() != 0 || mMultiTouchScale != 1.0f) {
                mRotateScalePoints[0] = x;
                mRotateScalePoints[1] = y;
                mUnrotateAndScaleMatrix.mapPoints(mRotateScalePoints);
                reuse.set((int) mRotateScalePoints[0], (int) mRotateScalePoints[1]);
            } else
                reuse.set(x, y);
            return reuse;
        }

        /**
         * This will apply the current map's scaling and rotation for a point. This can be useful when
         * converting MotionEvents to a screen point.
         */
        public PointF rotateAndScalePoint(int x, int y, PointF reuse) {
            if (reuse == null)
                reuse = new PointF();
            if (getMapOrientation() != 0 || mMultiTouchScale != 1.0f) {
                mRotateScalePoints[0] = x;
                mRotateScalePoints[1] = y;
                mRotateAndScaleMatrix.mapPoints(mRotateScalePoints);
                reuse.set((int) mRotateScalePoints[0], (int) mRotateScalePoints[1]);
            } else
                reuse.set(x, y);
            return reuse;
        }


        // From old projection!

        /**
         * Returns a value that lies within <code>minValue</code> and <code>maxValue</code> by
         * subtracting/adding <code>interval</code>.
         *
         * @param n the input number
         * @param minValue the minimum value
         * @param maxValue the maximum value
         * @param interval the interval length
         * @return a value that lies within <code>minValue</code> and <code>maxValue</code> by
         * subtracting/adding <code>interval</code>
         */
        private static double wrap(double n, final double minValue, final double maxValue,
                                   final double interval) {
            if (minValue > maxValue) {
                throw new IllegalArgumentException(
                        "minValue must be smaller than maxValue: " + minValue + ">" + maxValue);
            }
            if (interval > maxValue - minValue + 1) {
                throw new IllegalArgumentException(
                        "interval must be equal or smaller than maxValue-minValue: "
                                + "min: "
                                + minValue
                                + " max:"
                                + maxValue
                                + " int:"
                                + interval
                );
            }
            while (n < minValue) {
                n += interval;
            }
            while (n > maxValue) {
                n -= interval;
            }
            return n;
        }

        /**
         * Clips a number to the specified minimum and maximum values.
         *
         * @param n The number to clip
         * @param minValue Minimum allowable value
         * @param maxValue Maximum allowable value
         * @return The clipped value.
         */
        private static double clip(final double n, final double minValue, final double maxValue) {
            return Math.min(Math.max(n, minValue), maxValue);
        }

        public static void setTileSize(final int tileSize) {
            mTileSize = tileSize;
        }

        public static int getTileSize() {
            return mTileSize;
        }

        /**
         * Determines the map width and height (in pixels) at a specified level of detail.
         *
         * @param levelOfDetail Level of detail, from 1 (lowest detail) to 23 (highest detail)
         * @return The map width and height in pixels
         */
        public static int mapSize(final float levelOfDetail) {
            return (int) (GeometryMath.leftShift(mTileSize, levelOfDetail));
        }

        /**
         * Determines the ground resolution (in meters per pixel) at a specified latitude and level of
         * detail.
         *
         * @param latitude Latitude (in degrees) at which to measure the ground resolution
         * @param levelOfDetail Level of detail, from 1 (lowest detail) to 23 (highest detail)
         * @return The ground resolution, in meters per pixel
         */
        public static double groundResolution(final double latitude, final float levelOfDetail) {
            double result = wrap(latitude, -90, 90, 180);
            result = clip(result, MIN_LATITUDE, MAX_LATITUDE);
            return Math.cos(result * Math.PI / 180) * 2 * Math.PI * RADIUS_EARTH_METERS / mapSize(
                    levelOfDetail);
        }

        /**
         * Determines the ground resolution (in meters per pixel) at a specified latitude and level of
         * detail.
         *
         * @param latitude Latitude (in degrees) at which to measure the ground resolution
         * @return The ground resolution, in meters per pixel
         */
        public double groundResolution(final double latitude) {
            return groundResolution(latitude, mZoomLevelProjection);
        }

        /**
         * Converts a pixel from pixel XY coordinates at a specified level of detail into
         * latitude/longitude WGS-84 coordinates (in degrees).
         *
         * @param pixelX        X coordinate of the point, in pixels
         * @param pixelY        Y coordinate of the point, in pixels
         * @param levelOfDetail Level of detail, from 1 (lowest detail) to 23 (highest detail)
         * @return Output parameter receiving the latitude and longitude in degrees.
         */
        public static LatLng pixelXYToLatLong(double pixelX, double pixelY, final float levelOfDetail) {
            final double mapSize = mapSize(levelOfDetail);
            final double maxSize = mapSize - 1.0;
            double x = wrap(pixelX, 0, maxSize, mapSize);
            double y = wrap(pixelY, 0, maxSize, mapSize);

            x = (clip(x, 0, maxSize) / mapSize) - 0.5;
            y = 0.5 - (clip(y, 0, maxSize) / mapSize);

            final double latitude = 90.0 - 360.0 * Math.atan(Math.exp(-y * 2 * Math.PI)) / Math.PI;
            final double longitude = 360.0 * x;

            return new LatLng(latitude, longitude);
        }

        /**
         * Converts a pixel from pixel XY coordinates at a specified level of detail into
         * latitude/longitude WGS-84 coordinates (in degrees).
         *
         * @param pixelX X coordinate of the point, in pixels
         * @param pixelY Y coordinate of the point, in pixels
         * @return Output parameter receiving the latitude and longitude in degrees.
         */
        public LatLng pixelXYToLatLong(double pixelX, double pixelY) {
            return pixelXYToLatLong(pixelX, pixelY, mZoomLevelProjection);
        }

        /**
         * Converts a point from latitude/longitude WGS-84 coordinates (in degrees) into pixel XY
         * coordinates at a specified level of detail.
         *
         * @param latitude Latitude of the point, in degrees
         * @param longitude Longitude of the point, in degrees
         * @param levelOfDetail Level of detail, from 1 (lowest detail) to 23 (highest detail)
         * @param reuse An optional Point to be recycled, or null to create a new one automatically
         * @return Output parameter receiving the X and Y coordinates in pixels
         */
        public static PointF latLongToPixelXY(double latitude, double longitude,
                                              final float levelOfDetail, final PointF reuse) {
            latitude = wrap(latitude, -90, 90, 180);
            longitude = wrap(longitude, -180, 180, 360);
            final PointF out = (reuse == null ? new PointF() : reuse);

            latitude = clip(latitude, MIN_LATITUDE, MAX_LATITUDE);
            longitude = clip(longitude, MIN_LONGITUDE, MAX_LONGITUDE);

            final double x = (longitude + 180) / 360;
            final double sinLatitude = Math.sin(latitude * Math.PI / 180);
            final double y = 0.5 - Math.log((1 + sinLatitude) / (1 - sinLatitude)) / (4 * Math.PI);

            final float mapSize = mapSize(levelOfDetail);
            out.x = (float) clip(x * mapSize, 0, mapSize - 1);
            out.y = (float) clip(y * mapSize, 0, mapSize - 1);
            return out;
        }

        /**
         * Converts pixel XY coordinates into tile XY coordinates of the tile containing the specified
         * pixel.
         *
         * @param pixelX Pixel X coordinate
         * @param pixelY Pixel Y coordinate
         * @param reuse An optional Point to be recycled, or null to create a new one automatically
         * @return Output parameter receiving the tile X and Y coordinates
         */
        public static Point pixelXYToTileXY(final int pixelX, final int pixelY, final Point reuse) {
            final Point out = (reuse == null ? new Point() : reuse);
            out.x = pixelX / mTileSize;
            out.y = pixelY / mTileSize;
            return out;
        }

        /**
         * Converts tile XY coordinates into pixel XY coordinates of the upper-left pixel of the
         * specified tile.
         *
         * @param tileX
         * Tile X coordinate
         * @param tileY
         * Tile X coordinate
         * @param reuse
         * An optional Point to be recycled, or null to create a new one automatically
         * @return Output parameter receiving the pixel X and Y coordinates
         */
        public static PointF tileXYToPixelXY(final float tileX, final float tileY, final PointF reuse) {
            final PointF out = (reuse == null ? new PointF() : reuse);
            out.x = tileX * mTileSize;
            out.y = tileY * mTileSize;
            return out;
        }
    }
