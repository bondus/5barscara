


function fsquare(a) { return a*a }
function sqrtf(a)   { return Math.sqrt(a) }
function fabs(a)    { return Math.abs(a) }


class FiveBarScaraKinematics {
    constructor() {
        this.xOrigL = -50//-54
        this.xOrigR =  50 //54 
        
        this.yOrigL = 0 //-170
        this.yOrigR = 0 //-170 
        
        this.workmode = 2
        
        this.proximalL = 100.0 //128
        this.proximalR = 100.0 //128 //127.5
        
        this.distalL = 100 //182.0
        this.distalR = 100 //182.0 //181.0
        
        this.constrMin = 0
        this.constrMax = 180

        this.proxDistLAngleMin = 0
        this.proxDistLAngleMax = 360
        this.proxDistRAngleMin = 0
        this.proxDistRAngleMax = 360
        
        this.actuatorAngleLMin =  0
        this.actuatorAngleLMax =  360
        
        this.actuatorAngleRMin =  0
        this.actuatorAngleRMax =  360
        
        this.printAreaDefined = false

    }

    
    // quadrants: 1 is right upper, 2 is left upper, 3 is left down, 4 is right down
    getQuadrant(x, y)
    {
        if(x >= 0 && y >= 0) {
            return 1
        }
        else if(x < 0 && y >= 0) {
            return 2
        }
        else if(x < 0 && y < 0) {
            return 3
        }
        else { // x >= 0 && y < 0
            return 4
        }
    }
    
    // get angle between 0 and 360 for given origin and destination coordinates
    getAbsoluteAngle(xOrig, yOrig, xDest, yDest)
    {
        var length = sqrtf(fsquare(xOrig - xDest) + fsquare(yOrig - yDest))
        
        var y = fabs(yOrig - yDest)
        var angle = Math.asin(y / length) * 180.0 / Math.PI

        var quad = this.getQuadrant(xDest-xOrig, yDest-yOrig)

        if(quad == 1) {  // right upper quadrant I
            // nothing to change
        }
        else if(quad == 2) {  // left upper quadrant II
            angle = 180.0 - angle
        }
        else if(quad == 3) { // left lower quadrant III
            angle += 180.0
        }
        else if(quad == 4) { // right lower quadrant IV
            angle = 360 - angle 
        }
        return angle
    }

    // return negative if turn is left (counter-clockwise), positive if turn is right (clockwise)
    getTurn( x1,  y1, xAngle, yAngle, x2, y2)
    {
        return ((xAngle-x1)*(y2-y1))-((yAngle-y1)*(x2-x1))
    }
    
    // 1 - angle - 2 are ordered clockwise. The angle is at the inner/right side, between 2 and 1 clockwise
    getAngle( x1,  y1,  xAngle,  yAngle,  x2,  y2)
    {
        var angle1 = this.getAbsoluteAngle(xAngle, yAngle, x1, y1);
        var angle2 = this.getAbsoluteAngle(xAngle, yAngle, x2, y2);

        var angle = 0.0;
        if(angle2 < angle1) {
            angle = 360 + angle2 - angle1;
        }
        else {
            angle = angle2 - angle1;
        }
        
        return angle;
    }


    
    // first circle, second circle. Return the two intersection points
    getIntersec(firstRadius, secondRadius,  firstX,  firstY, secondX,  secondY)
    {
        var firstRadius2  = fsquare(firstRadius)
        var secondRadius2 = fsquare(secondRadius)

        var distance2 = fsquare(firstX - secondX) + fsquare(firstY - secondY)
        var distance = sqrtf(distance2)

        var delta = 0.25 * sqrtf(
            (distance + firstRadius + secondRadius)
                * (distance + firstRadius - secondRadius)
                * (distance - firstRadius + secondRadius)
                * (-distance + firstRadius + secondRadius)
        )

        // calculate x
        var term1x = (firstX + secondX) / 2
        var term2x = (secondX - firstX) * (firstRadius2 - secondRadius2) / (2 * distance2)
        var term3x = 2 * (firstY - secondY) / (distance2) * delta
        var x1 = term1x + term2x + term3x
        var x2 = term1x + term2x - term3x

        // calculate y
        var term1y = (firstY + secondY) / 2
        var term2y = (secondY - firstY)*(firstRadius2 - secondRadius2) / (2 * distance2)
        var term3y = 2 * (firstX - secondX) / distance2 * delta
        var y1 = term1y + term2y - term3y
        var y2 = term1y + term2y + term3y

        return [x1, y1, x2, y2]
    }
    
    
    getTheta( prox,  distal,  proxX, proxY,  destX,  destY,  isRightArm)
    {
        var [x1, y1, x2, y2] = this.getIntersec(prox, distal, proxX, proxY, destX, destY)

        var thetaA = this.getAbsoluteAngle(proxX, proxY, x1, y1)
        var thetaB = this.getAbsoluteAngle(proxX, proxY, x2, y2)
        
        var proxTurnA = this.getTurn(proxX, proxY, x1, y1, destX, destY)
        
        var proxTurnB = this.getTurn(proxX, proxY, x2, y2, destX, destY)
        
        var use = 0 // 1 for A, 2 for B

        if(this.workmode == 1) {
            if(proxTurnA > 0) use = 1
            else if(proxTurnB > 0) use = 2
        } else if (this.workmode == 2) {
            if(!isRightArm) {
                if(proxTurnA < 0) use = 1
                else if(proxTurnB < 0) use = 2
            } else {
                if(proxTurnA > 0) use = 1
                else if(proxTurnB > 0) use = 2            
            }
        } else if (this.workmode == 3) {
            if(!isRightArm) {
                if(proxTurnA > 0) use = 1
                else if(proxTurnB > 0) use = 2
            } else {
                if(proxTurnA < 0) use = 1
                else if(proxTurnB < 0) use = 2            
            }
        } else if (this.workmode == 4) {
            if(proxTurnA < 0) use = 1
            else if(proxTurnB < 0) use = 2
        }
    
        if(use == 1)
        {
            return [x1, y1, thetaA, x2, y2, thetaB]
        }
        else if(use == 2)
        {
            return [x2, y2, thetaB, x1, y1, thetaA]
        }
        else
        {
            return [Number.Nan, Number.Nan, Number.Nan, Number.Nan, Number.Nan, Number.Nan]
        }
    }
    
    getXYFromAngle( angle,  length, origX, origY)
    {
        var xL = length * Math.cos(angle * Math.PI / 180.0);
        var yL = length * Math.sin(angle * Math.PI / 180.0);

        return [xL + origX, yL + origY]
    }
    
    getInverse(x, y) {
        
        var lefttheta = this.getTheta(this.proximalL, this.distalL, this.xOrigL, this.yOrigL, x, y, false)
        var thetaL = lefttheta[2]
        
        var righttheta = this.getTheta(this.proximalR, this.distalR, this.xOrigR, this.yOrigR, x, y, true)
        var thetaR = righttheta[2]

        // Sanity check hotend turn
        var turnHot = this.getTurn(lefttheta[0], lefttheta[1],  x, y, righttheta[0], righttheta[1]);

        if(turnHot >0 ) { // left turn!!
            return [Math.NaN, Math.NaN]
        }

        // TODO: cantilevering
        
        return [thetaL, thetaR]
    }

    getForward( thetaL,  thetaR)
    {
  
        var [xL, yL] = this.getXYFromAngle(thetaL, this.proximalL, this.xOrigL, this.yOrigL);

        var [xR, yR] = this.getXYFromAngle(thetaR, this.proximalR, this.xOrigR, this.yOrigR);

        var [xDst1, yDst1, xDst2, yDst2] = this.getIntersec(this.distalL, this.distalR, xL, yL, xR, yR);
        
        // Figure out what solution to pick, depending on angle of hotend joint
        var turnHot1 = this.getTurn(xL, yL, xDst1, yDst1, xR, yR);
        var turnHot2 = this.getTurn(xL, yL, xDst2, yDst2, xR, yR);
        
        var xDst = Math.Nan
        var yDst = Math.Nan
        if(this.workmode >= 1 && this.workmode <= 4) {
            if(turnHot1 < 0) { //
                xDst = xDst1
                yDst = yDst1
            }
            else if(turnHot2 < 0) {
                xDst = xDst2
                yDst = yDst2
            }
            // Sanity check the elbow joins to make sure it's in the correct work mode
            var tL = this.getTurn(this.xOrigL, this.yOrigL, xL, yL, xDst, yDst);
            var tR = this.getTurn(this.xOrigR, this.yOrigR, xR, yR, xDst, yDst);
            
            if((this.workmode == 1 && (tL < 0 || tR < 0)) ||
               (this.workmode == 2 && (tL > 0 || tR < 0)) ||
               (this.workmode == 3 && (tL < 0 || tR > 0)) ||
               (this.workmode == 4 && (tL > 0 || tR > 0))) {
                xDst = Math.Nan
                yDst = Math.Nan
            }
            
        } else {
            // TODO work mode 5-8
        }
        
        return [xL, yL, xR, yR, xDst, yDst]
    }

    isReachable(x, y)
    {
        var lefttheta = this.getTheta(this.proximalL, this.distalL, this.xOrigL, this.yOrigL, x, y, false)
        var thetaL = lefttheta[2]
        
        var righttheta = this.getTheta(this.proximalR, this.distalR, this.xOrigR, this.yOrigR, x, y, true)
        var thetaR = righttheta[2]
        
    }

    constraintsOk(x,y)
    {
        //return true; // XXX
        var lefttheta = this.getTheta(this.proximalL, this.distalL, this.xOrigL, this.yOrigL, x, y, false)
        var thetaL = lefttheta[2]
        var XL = lefttheta[0]
        var YL = lefttheta[1]
        
        var righttheta = this.getTheta(this.proximalR, this.distalR, this.xOrigR, this.yOrigR, x, y, true)
        var thetaR = righttheta[2]
        var XR = righttheta[0]
        var YR = righttheta[1]

        //console.log(thetaL, thetaR);
        if(isNaN(thetaL) || isNaN(thetaR)) return false;
        
        // check theta angles
        if(this.actuatorAngleLMin < 0 &&  thetaL > this.actuatorAngleLMax) thetaL -= 360;
        if(this.actuatorAngleLMin > thetaL || this.actuatorAngleLMax < thetaL) {
            return false;
        }
        
        if(this.actuatorAngleRMin < 0 &&  thetaR > this.actuatorAngleRMax) thetaR -= 360;
        if(this.actuatorAngleRMin > thetaR || this.actuatorAngleRMax < thetaR) {
            return false;
        }


        
        // check constr
        var constr = this.getAngle(XL, YL, x, y, XR, YR);
        //printf("c: %f\n", constr);
        if(this.constrMin > constr || this.constrMax < constr) {
            return false;
        }
        
        // check proxDistal angle L and R
        var angleProxDistL = this.getAngle(this.xOrigL, this.yOrigL, XL, YL, x, y);
        //console.log(angleProxDistL, this.proxDistLAngleMax)
        if(this.proxDistLAngleMin > angleProxDistL || this.proxDistLAngleMax < angleProxDistL) {
            return false;
        }
        var angleProxDistR = this.getAngle(this.xOrigR, this.yOrigR, XR, YR, x, y);
        //printf("aR: %f\n", angleProxDistR);
        //console.log(angleProxDistR, this.proxDistRAngleMax)
        if(this.proxDistRAngleMin > angleProxDistR || this.proxDistRAngleMax < angleProxDistR) {
            return false;
        }
        
        return true;
    }    
}

function drawCanvas(draw) {

    var pattern = draw.pattern(100, 100, function(add) {
        add.line(0,0, 100,0).stroke({width:0.5})
        add.line(0,10,100,10).stroke({width:0.1})
        add.line(0,20,100,20).stroke({width:0.1})
        add.line(0,30,100,30).stroke({width:0.1})
        add.line(0,40,100,40).stroke({width:0.1})
        add.line(0,50,100,50).stroke({width:0.1})
        add.line(0,60,100,60).stroke({width:0.1})
        add.line(0,70,100,70).stroke({width:0.1})
        add.line(0,80,100,80).stroke({width:0.1})
        add.line(0,90,100,90).stroke({width:0.1})
        add.line(0, 0,0,100).stroke({width:0.5})
        add.line(10,0,10,100).stroke({width:0.1})
        add.line(20,0,20,100).stroke({width:0.1})
        add.line(30,0,30,100).stroke({width:0.1})
        add.line(40,0,40,100).stroke({width:0.1})
        add.line(50,0,50,100).stroke({width:0.1})
        add.line(60,0,60,100).stroke({width:0.1})
        add.line(70,0,70,100).stroke({width:0.1})
        add.line(80,0,80,100).stroke({width:0.1})
        add.line(90,0,90,100).stroke({width:0.1})
    })

    draw.rect(800,800).fill(pattern);


}

function drawScara(draw, fbs, step, workmode) {

    var xs = Array(360).fill(0).map(x => Array(360).fill(0))
    var ys = Array(360).fill(0).map(x => Array(360).fill(0))

    var lines_l = Array(360).fill(0).map(x => Array(360).fill(0))
    var lines_r = Array(360).fill(0).map(x => Array(360).fill(0))

    fbs.workmode = workmode
    for(l = 0; l < 360; l += step) {
        for(r = 0; r < 360; r += step) {
            var [xL, yL, xR, yR, xDst, yDst] = fbs.getForward(l, r)
            xs[l][r] = xDst
            ys[l][r] = yDst
        }        
    }
    
    var color = "#000"
    if(workmode == 1) color = "#700"
    if(workmode == 2) color = "#070"
    if(workmode == 3) color = "#707"
    if(workmode == 4) color = "#007"
    
    for(l = 0; l < 360; l += step) {
        for(r = 0; r < 360; r += step) {
            var x0 = xs[l][r];
            var y0 = ys[l][r];
            var ok = fbs.constraintsOk(x0, y0)
            var opacity = 1
            if(!ok) opacity = 0.2
            if(!isNaN(x0) && !isNaN(y0))
            {                
                var ll = (l+step) % 360
                var x1 = xs[ll][r];
                var y1 = ys[ll][r];
                if(!isNaN(x1) && !isNaN(y1))
                {
                    var line = draw.line(400+x0,400-y0, 400+x1,400-y1).stroke({ width: 0.3, color: color, opacity:opacity});
                }
                var rr = (r+step) % 360
                var x2 = xs[l][rr];
                var y2 = ys[l][rr];
                if(!isNaN(x2) && !isNaN(y2))
                {
                    draw.line(400+x0,400-y0, 400+x2,400-y2).stroke({ width: 0.3, color: color, opacity:opacity});
                }
            }
            
        }        
    }
    /*
    var [l, r] = fbs.getInverse(0,80)    
    var [xL, yL, xR, yR, xDst, yDst] = fbs.getForward(l, r)
      */  
}

function inputChanged() {

    var draw = document.draw
    var fbs  = document.fbs

    var proxL = parseFloat(document.getElementById("proxL").value)
    if(!isNaN(proxL)) fbs.proximalL = proxL
    document.getElementById("proxL").value = fbs.proximalL

    var proxR = parseFloat(document.getElementById("proxR").value)
    if(!isNaN(proxR)) fbs.proximalR = proxR
    document.getElementById("proxR").value = fbs.proximalR

    var distL = parseFloat(document.getElementById("distL").value)
    if(!isNaN(distL)) fbs.distalL = distL
    document.getElementById("distL").value = fbs.distalL

    var distR = parseFloat(document.getElementById("distR").value)
    if(!isNaN(distR)) fbs.distalR = distR
    document.getElementById("distR").value = fbs.distalR

    var actDist = document.getElementById("actDistance").value
    if(!isNaN(actDist)) {
        fbs.xOrigL = -actDist/2
        fbs.xOrigR = actDist/2
    }
    document.getElementById("actDistance").value = fbs.xOrigR - fbs.xOrigL
        
    draw.clear()
    drawCanvas(draw)

    for(workmode = 1; workmode <= 4; workmode ++) {
        var color = "#000"
        if(workmode == 1) color = "#f00"
        if(workmode == 2) color = "#0f0"
        if(workmode == 3) color = "#f0f"
        if(workmode == 4) color = "#00f"
        document.proxL[workmode]=draw.line(0, 0, 0, 0).stroke({ width: 2, color: color  })
        document.proxR[workmode]=draw.line(0, 0, 0, 0).stroke({ width: 2, color: color  })
        document.distL[workmode]=draw.line(0, 0, 0, 0).stroke({ width: 2, color: color  })
        document.distR[workmode]=draw.line(0, 0, 0, 0).stroke({ width: 2, color: color  })

        document.resolution[workmode] = draw.text(""); 
    }

    document.coords = draw.text("X Y").move(0,16)
    
    draw.circle(10).move(400+fbs.xOrigL-5, 400-fbs.yOrigL-5)
    draw.circle(10).move(400+fbs.xOrigR-5, 400-fbs.yOrigR-5)
    
    if(document.getElementById("work1").checked) drawScara(draw, fbs, 4, 1)
    if(document.getElementById("work2").checked) drawScara(draw, fbs, 4, 2)
    if(document.getElementById("work3").checked) drawScara(draw, fbs, 4, 3)
    if(document.getElementById("work4").checked) drawScara(draw, fbs, 4, 4)     
}

function clicked(event) {


    var draw = document.draw
    var fbs  = document.fbs

    var x  = event.offsetX - 400
    var y = -(event.offsetY - 400)


    for(workmode = 1; workmode <= 4; workmode ++) {
        var name  = "work"+workmode        
        var active = document.getElementById(name).checked
        fbs.workmode = workmode;        
        var [l, r] = fbs.getInverse(x,y)        
        if(fbs.constraintsOk(x, y) && active && !isNaN(l) && !isNaN(r)) {
            var [xL, yL, xR, yR, xDst, yDst ]= fbs.getForward(l, r)
            document.proxL[workmode].attr({x1 : 400+fbs.xOrigL, y1: 400-fbs.yOrigL, x2:400+xL, y2:400-yL})
            document.proxR[workmode].attr({x1 : 400+fbs.xOrigR, y1: 400-fbs.yOrigR, x2:400+xR, y2:400-yR})
            document.distL[workmode].attr({x1 : 400+xL, y1: 400-yL, x2:400+xDst, y2:400-yDst})
            document.distR[workmode].attr({x1 : 400+xR, y1: 400-yR, x2:400+xDst, y2:400-yDst})            
            
            var [_, _, _, _, xl, yl ] = fbs.getForward(l+0.1, r);
            var [_, _, _, _, xr, yr ] = fbs.getForward(l, r+0.1);
            var rl = Math.sqrt(fsquare(xl-x) + fsquare(yl-y))/0.1
            var rr = Math.sqrt(fsquare(xr-x) + fsquare(yr-y))/0.1
            //console.log(document.resolution[workmode])
            //document.resolution[workmode].text("wm"+workmode+": " + rl.toFixed(2) + " " + rr.toFixed(2) + " mm/deg")

            var stepsPerDeg = 106.666;
            rl = stepsPerDeg / rl
            rr = stepsPerDeg / rr
            document.resolution[workmode].text("L "+l.toFixed(1) + " R " + r.toFixed(1)).move(0,workmode*16+16)
            //document.resolution[workmode].text("wm"+workmode+": " + rl.toFixed(2) + " " + rr.toFixed(2) + " s/mm")
            
        } else {
            document.proxL[workmode].attr({x1 : 0, y1: 0, x2:40, y2:0})
            document.proxR[workmode].attr({x1 : 0, y1: 0, x2:40, y2:0})
            document.distL[workmode].attr({x1 : 0, y1: 0, x2:40, y2:0})
            document.distR[workmode].attr({x1 : 0, y1: 0, x2:40, y2:0})
        }

    }

    document.coords.text("X " + x + " Y " + y)
    

    
}

SVG.on(document, 'DOMContentLoaded', function() {
    var draw = SVG('drawing').size(800, 800)
    var fbs = new FiveBarScaraKinematics()

    document.getElementById("proxL").value = fbs.proximalL
    document.getElementById("proxR").value = fbs.proximalR

    document.getElementById("distL").value = fbs.distalL
    document.getElementById("distR").value = fbs.distalR

    document.getElementById("actDistance").value = fbs.xOrigR - fbs.xOrigL

    document.getElementById("work1").checked = fbs.workmode == 1;
    document.getElementById("work2").checked = fbs.workmode == 2;
    document.getElementById("work3").checked = fbs.workmode == 3;
    document.getElementById("work4").checked = fbs.workmode == 4;

    document.fbs = fbs;
    document.draw = draw;

    // For the line objects
    document.proxL = Array(4).fill(null)
    document.proxR = Array(4).fill(null)
    document.distL = Array(4).fill(null)
    document.distR = Array(4).fill(null)

    document.resolution = Array(4).fill(null)


    
    inputChanged()
})


