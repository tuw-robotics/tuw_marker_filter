#include "tuw_geometry/linesegment2d_detector.h"

using namespace tuw;

LineSegment2DDetector::LineSegment2DDetector(){
}
void LineSegment2DDetector::LineSegment::updatePoints(const std::vector<Point2D> &points) {
    if((idx0_ < idx1_) && (idx0_ >= 0) && (idx1_ <  points.size())) {
        points_.clear();
        points_.reserve(idx1_-idx0_+1);
        for(unsigned int i = idx0_; i <= idx1_; i++) {
            points_.push_back(points[i]);
        }
    }
}
bool LineSegment2DDetector::LineSegment::isSupportPoint(int idx) {
    if( (idx < idx0_) || (idx > idx1_)) {
        return false;
    } else {
        return true;
    }
}
unsigned int LineSegment2DDetector::LineSegment::nrSupportPoint() {
    return idx1_ - idx0_ + 2;
}
void LineSegment2DDetector::LineSegment::set(unsigned int idx0, unsigned int idx1, const std::vector<Point2D> &points) {
    idx0_ = idx0, idx1_ = idx1;
    LineSegment2D::set(points[idx0], points[idx1]);
}


void LineSegment2DDetector::start (const std::vector<Point2D> &points) {

    connected_measurments_.clear();
    segments_.clear();

    if(points.size() > 0) {

        std::pair< unsigned int, unsigned int> idx;
        idx.first = 0;

        while(idx.first < points.size()) {
            idx.second = idx.first + 1;
            float threshold = 4 * points[idx.second].distanceTo(points[idx.second+1]);
            while(idx.second < points.size()) {
                if(config_.threshold_split_neighbor) {	    
                    float d = points[idx.second].distanceTo(points[idx.second+1]);
                    if(d > threshold) {
                        break;
                    }
                    threshold = 4 * d;
                }
                idx.second++;
            }
            if((idx.second - idx.first) > 2) {
                connected_measurments_.push_back(idx);
            }
            idx.first = idx.second+1;
        }

        for(unsigned int i = 0; i < connected_measurments_.size(); i++) {
            unsigned int idx0 = connected_measurments_[i].first;
            unsigned int idx1 = connected_measurments_[i].second;
            if(idx1 > idx0) {
                LineSegment line;
                line.set(idx0, idx1, points);
                split(line, points);
            }
        }
    }
}
std::vector<LineSegment2D> &LineSegment2DDetector::start (const std::vector<Point2D> &points, std::vector<LineSegment2D> &detected_segments){
  start(points);
  detected_segments.reserve(detected_segments.size() + segments_.size());
  for(const LineSegment &l: segments_){
    detected_segments.push_back(l);
  }  
  return detected_segments;
}
void LineSegment2DDetector::split(LineSegment &line, const std::vector<Point2D> &points) {
    unsigned int idxMax=line.idx0_;
    float d;
    float dMax = 0;
    for(unsigned int i = line.idx0_; i < line.idx1_; i++) {
        d = fabs(line.distanceTo(points[i]));
        if(d > dMax) {
            dMax = d,  idxMax = i;
        }
    }
    if(dMax > config_.threshold_split) {
        LineSegment l0,l1;
        if(line.idx0_ + config_.min_points_per_line < idxMax) {
            l0.set(line.idx0_, idxMax, points);
            split(l0, points);
        }
        if(idxMax + config_.min_points_per_line < line.idx1_) {
            l1.set(idxMax, line.idx1_, points);
            split(l1, points);
        }
    } else {
        if(line.length() < config_.min_length) {
            return;
        }
        if(((float) line.nrSupportPoint()) / line.length() < config_.min_points_per_unit) {
            return;
        }
        line.id_ = segments_.size();
        line.updatePoints(points);
        segments_.push_back(line);
    }
}

const std::vector<LineSegment2DDetector::LineSegment> & LineSegment2DDetector::result(){
  return segments_;
}