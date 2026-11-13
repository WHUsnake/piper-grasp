"""Adapt GraspNet orientations to observed bottle geometry and PiPER pad centre.

Only rendered target points are used for planning; MuJoCo object poses are used
elsewhere solely for physical evaluation. Assumes the experiment's upright bottle.
"""
import numpy as np

PAD_CENTER_FROM_LINK6_M=.105
FINGER_TIP_FROM_LINK6_M=.13503


def fit_bottle(points):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or not np.isfinite(points).all():
        raise ValueError('Insufficient finite bottle points for contact-centre fitting')
    low,high=np.percentile(points[:,2],[.5,99.5]);span=float(high-low)
    if span<.06:raise ValueError('Insufficient observed bottle height')
    # Exclude neck/cap and bottom edges; fit the cylindrical body from visible depth.
    body=points[(points[:,2]>low+.20*span)&(points[:,2]<low+.65*span)]
    a=np.column_stack([2*body[:,0],2*body[:,1],np.ones(len(body))])
    b=np.sum(body[:,:2]**2,axis=1)
    coef,_,rank,_=np.linalg.lstsq(a,b,rcond=None)
    radius=float(np.sqrt(max(0,coef[2]+np.sum(coef[:2]**2))))
    if rank<3 or not .008<radius<.04:raise ValueError('Invalid observed cylinder fit')
    residual=np.linalg.norm(body[:,:2]-coef[:2],axis=1)-radius
    if np.sqrt(np.mean(residual**2))>.005:raise ValueError('Bottle body does not support a stable cylinder fit')
    return dict(axis_xy=coef[:2].tolist(),radius_m=radius,bottom_observed_z_m=float(low),
                top_observed_z_m=float(high),observed_height_m=span,
                preferred_contact_height_fraction=.50,fit_rmse_m=float(np.sqrt(np.mean(residual**2))))


def fit_near_sphere(points,support_surface_z=None):
    """Infer the hidden half of a round target from its segmented visible surface."""
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or not np.isfinite(points).all():
        raise ValueError('Insufficient finite target points')
    low,high=np.percentile(points,[1,99],axis=0)
    span=high-low
    if np.any(span<.008) or np.any(span>.13):
        raise ValueError('Target extent is outside the supported near-spherical range')
    a=np.column_stack([2*points,np.ones(len(points))]);b=np.sum(points**2,axis=1)
    coef,_,rank,_=np.linalg.lstsq(a,b,rcond=None)
    centre=coef[:3];radius=float(np.sqrt(max(0,coef[3]+np.sum(centre**2))))
    residual=np.linalg.norm(points-centre,axis=1)-radius
    if rank<4 or not .015<radius<.065 or np.sqrt(np.mean(residual**2))>.008:
        raise ValueError('Target cloud does not support a stable near-sphere fit')
    radius+=.0005
    if support_surface_z is not None:
        # The MoveIt collision primitive adds 1 mm. Keep an object resting on
        # the support plane out of initial collision without shrinking it.
        centre[2]=max(centre[2],float(support_surface_z)+radius+.0015)
    return dict(shape='sphere',center=centre.tolist(),radius_m=radius,
                observed_height_m=float(span[2]),observed_extent_m=span.tolist(),
                fit_rmse_m=float(np.sqrt(np.mean(residual**2))))


def refine_near_sphere_candidates(candidates,points,tcp_m,max_opening_m=.075,support_surface_z=None):
    fit=fit_near_sphere(points,support_surface_z);centre=np.asarray(fit['center']);opening=2*fit['radius_m']+.004
    if opening>max_opening_m:
        raise ValueError(f'Target requires {opening:.3f} m opening; gripper limit is {max_opening_m:.3f} m')
    output=[]
    for original in candidates:
        c=dict(original);approach=np.asarray(c['rotation_world'])[:,2]
        old=np.asarray(c['position']);target=centre+approach*(tcp_m-PAD_CENTER_FROM_LINK6_M)
        c.update(network_tcp_position_world=old.tolist(),position=target.tolist(),
                 contact_center_world=centre.tolist(),geometry_refined=True,
                 geometry_opening_m=opening,network_opening_m=float(c['width']),width=max(float(c['width']),opening),
                 selection_score=float(c['score']-.5*np.linalg.norm(target-old)),
                 orientation_adaptation='near_sphere_network_direction')
        if c['width']<=max_opening_m:output.append(c)
    output.sort(key=lambda c:c['selection_score'],reverse=True)
    return output,fit


def refine_box_candidates(candidates,points,tcp_m,size_m,support_surface_z,max_opening_m=.075):
    """Fit a front-visible, axis-aligned box using its declared size as a prior.

    A single depth view cannot determine the hidden rear face. The configured
    box dimensions are therefore an explicit simulation prior, not an oracle
    pose or a replacement for GraspNet orientation proposals.
    """
    points=np.asarray(points,float);size=np.asarray(size_m,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100 or not np.isfinite(points).all():
        raise ValueError('Insufficient finite box points')
    if size.shape!=(3,) or np.any(size<=0) or np.any(size>.15):
        raise ValueError('Invalid box size prior')
    low,high=np.percentile(points,[1,99],axis=0)
    if high[2]-low[2]<.012 or high[2]-low[2]>size[2]+.015:
        raise ValueError('Observed box height inconsistent with size prior')
    # The depth camera views the negative-X face. Use its nearest observed
    # surface, the observed lateral midpoint, and the measured support plane.
    centre=np.array([float(np.percentile(points[:,0],10)+size[0]/2),
                     float((low[1]+high[1])/2),float(support_surface_z+size[2]/2+.001)])
    opening=float(size[1]+.010)
    if opening>max_opening_m:raise ValueError('Box exceeds gripper opening')
    fit=dict(shape='box',center=centre.tolist(),size_m=size.tolist(),
             radius_m=float(size[1]/2),observed_height_m=float(size[2]),
             observed_extent_m=(high-low).tolist(),size_prior=True)
    output=[]
    for original in candidates:
        c=dict(original);axis=np.asarray(c['rotation_world'])[:,2]
        old=np.asarray(c['position']);target=centre+axis*(tcp_m-PAD_CENTER_FROM_LINK6_M)
        c.update(network_tcp_position_world=old.tolist(),position=target.tolist(),
                 contact_center_world=centre.tolist(),geometry_refined=True,
                 geometry_opening_m=opening,network_opening_m=float(c['width']),width=max(float(c['width']),opening),
                 selection_score=float(c['score']-.5*np.linalg.norm(target-old)),
                 orientation_adaptation='box_network_direction')
        if c['width']<=max_opening_m:output.append(c)
    output.sort(key=lambda c:c['selection_score'],reverse=True)
    return output,fit


def refine_candidates(candidates,points,tcp_m):
    fit=fit_bottle(points);span=fit['observed_height_m'];bottom=fit['bottom_observed_z_m']
    target_z=bottom+fit['preferred_contact_height_fraction']*span
    output=[]
    for original in candidates:
        c=dict(original);r=np.asarray(c['rotation_world']);axis=r[:,2]
        if axis[0]<.7:continue
        old=np.asarray(c['position'])
        raw_reference=old-axis*(c['depth']-(.135-tcp_m))
        # Set the contact centre to the observed bottle middle. Preserve the
        # network orientation and retain all original proposal data for comparison.
        height_fraction=float((raw_reference[2]-bottom)/span)
        contact_z=float(target_z)
        centre=np.array([*fit['axis_xy'],contact_z])
        target=centre+axis*(tcp_m-PAD_CENTER_FROM_LINK6_M)
        c.update(network_tcp_position_world=old.tolist(),network_reference_world=raw_reference.tolist(),
                 position=target.tolist(),contact_center_world=centre.tolist(),
                 height_fraction=float((contact_z-bottom)/span),network_height_fraction=height_fraction,
                 refinement_translation_m=(target-old).tolist(),geometry_refined=True,
                 body_bottom_z_m=float(bottom),body_span_m=float(span),
                 reference='piper_effective_finger_pad_center',pad_center_from_link6_m=PAD_CENTER_FROM_LINK6_M,
                 tip_margin_target_m=FINGER_TIP_FROM_LINK6_M-PAD_CENTER_FROM_LINK6_M)
        c['geometry_opening_m']=float(2*fit['radius_m']+.012)
        c['network_opening_m']=float(c['width'])
        c['width']=float(max(c['width'],2*fit['radius_m']+.012))
        c['selection_score']=float(c['score']-.5*np.linalg.norm(target-old))
        output.append(c)
    output.sort(key=lambda c:c['selection_score'],reverse=True)
    return output,fit


def parallel_gripper_symmetries(candidates):
    """Same physical parallel-jaw grasp, two wrist-roll representations for IK."""
    from scipy.spatial.transform import Rotation
    output=list(candidates);flip=np.diag([-1.,-1.,1.])
    for original in candidates:
        symmetric=dict(original)
        symmetric['quaternion']=(Rotation.from_quat(original['quaternion'])*Rotation.from_matrix(flip)).as_quat().tolist()
        symmetric['rotation_world']=(np.asarray(original['rotation_world'])@flip).tolist()
        symmetric['parallel_gripper_symmetry']=True
        output.append(symmetric)
    return output


def upright_orientation_search(candidates,tcp_m,approach_origin=None,height_fractions=(.5,)):
    """Diverse cylinder hypotheses: network azimuth, radial approach, midpoint.

    Round-robin across network parents before adding wrist symmetries prevents
    one parent's variants from exhausting the planner's search budget.
    """
    from scipy.spatial.transform import Rotation
    families=[]
    for original in candidates[:5]:
        r=np.asarray(original['rotation_world']);azimuth=r[:,2].copy();azimuth[2]=0.
        length=np.linalg.norm(azimuth)
        if length<.7:continue
        azimuth/=length;centre=np.asarray(original['contact_center_world'])
        directions=[azimuth]
        if approach_origin is not None:
            radial=centre-np.asarray(approach_origin);radial[2]=0.
            if np.linalg.norm(radial)>1e-6:
                radial/=np.linalg.norm(radial)
                middle=radial+azimuth;middle/=np.linalg.norm(middle)
                directions=[radial,azimuth,middle]
        variants=[original]
        # Cover the main orientations before widening to contact-height alternatives.
        specs=[(elevation,az,.5) for elevation in [0.,-10.,10.,-20.,20.] for az in directions]
        if 'body_span_m' in original:
            alternatives=[(elevation,directions[0],float(fraction)) for elevation in [0.,-10.,10.] for fraction in height_fractions if abs(fraction-.5)>1e-6]
            specs=specs[:len(directions)+2]+alternatives+specs[len(directions)+2:]
        for elevation,az,fraction in specs:
            closing=np.cross([0.,0.,1.],az)
            angle=np.deg2rad(elevation);approach=az*np.cos(angle)+np.array([0.,0.,np.sin(angle)])
            rot=np.column_stack([np.cross(closing,approach),closing,approach])
            c=dict(original);contact=centre.copy()
            if 'body_span_m' in original:contact[2]=original['body_bottom_z_m']+fraction*original['body_span_m']
            c['width']=float(c.get('geometry_opening_m',c.get('width',.07)))
            c.update(rotation_world=rot.tolist(),quaternion=Rotation.from_matrix(rot).as_quat().tolist(),
                     position=(contact+approach*(tcp_m-PAD_CENTER_FROM_LINK6_M)).tolist(),contact_center_world=contact.tolist(),height_fraction=fraction,
                     orientation_adaptation='upright_cylinder_network_and_radial_search',adapted_elevation_deg=elevation,
                     network_rotation_world=r.tolist())
            variants.append(c)
        families.append(variants)
    output=[]
    # Geometry-adapted variants first, then retain unchanged parent poses.
    for index in list(range(1,max((len(f) for f in families),default=1)))+[0]:
        for family in families:
            if index<len(family):output.append(family[index])
    unique=[];rotations=[];positions=[];threshold=1+2*np.cos(np.deg2rad(4.))
    for candidate in output:
        rotation=np.asarray(candidate['rotation_world']);position=np.asarray(candidate['position'])
        if unique:
            close=np.linalg.norm(np.asarray(positions)-position,axis=1)<.005
            same=np.einsum('nij,ij->n',np.asarray(rotations),rotation)>threshold
            if np.any(close&same):continue
        unique.append(candidate);rotations.append(rotation);positions.append(position)
    return unique
