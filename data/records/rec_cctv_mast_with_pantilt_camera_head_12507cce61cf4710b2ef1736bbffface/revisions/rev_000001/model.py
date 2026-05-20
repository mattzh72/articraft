from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_pole_cctv_mast")

    concrete = model.material("weathered_concrete", rgba=(0.55, 0.54, 0.50, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.67, 1.0))
    dark_steel = model.material("dark_powder_coat", rgba=(0.07, 0.075, 0.08, 1.0))
    black = model.material("black_glass", rgba=(0.005, 0.006, 0.008, 1.0))
    amber = model.material("smoked_lens", rgba=(0.02, 0.025, 0.03, 0.92))

    mast = model.part("mast")

    # Square concrete footing and visible anchor hardware.
    mast.visual(
        Box((0.56, 0.56, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="concrete_base",
    )
    mast.visual(
        Cylinder(radius=0.125, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.2925)),
        material=galvanized,
        name="base_flange",
    )
    for ix, x in enumerate((-0.16, 0.16)):
        for iy, y in enumerate((-0.16, 0.16)):
            idx = ix * 2 + iy
            mast.visual(
                Cylinder(radius=0.010, length=0.070),
                origin=Origin(xyz=(x, y, 0.315)),
                material=dark_steel,
                name=f"anchor_stud_{idx}",
            )
            mast.visual(
                Cylinder(radius=0.026, length=0.010),
                origin=Origin(xyz=(x, y, 0.345)),
                material=dark_steel,
                name=f"anchor_nut_{idx}",
            )

    # A real street mast has a slight taper rather than a constant pipe.
    pole_height = 2.345
    pole_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.066, 0.0),
                (0.045, pole_height),
                (0.0, pole_height),
            ],
            segments=64,
            closed=True,
        ),
        "tapered_pole",
    )
    mast.visual(
        pole_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=galvanized,
        name="tapered_pole",
    )

    # Split clamp collar with bolted ears at the pole top.
    collar_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.048, -0.090),
                (0.078, -0.090),
                (0.078, 0.090),
                (0.048, 0.090),
            ],
            segments=64,
            closed=True,
        ),
        "clamp_collar",
    )
    mast.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.540)),
        material=dark_steel,
        name="clamp_collar",
    )
    for y in (-0.088, -0.126):
        mast.visual(
            Box((0.070, 0.030, 0.135)),
            origin=Origin(xyz=(0.0, y, 2.540)),
            material=dark_steel,
            name=f"clamp_ear_{abs(int(y * 1000))}",
        )
    for z in (2.495, 2.585):
        mast.visual(
            Cylinder(radius=0.012, length=0.100),
            origin=Origin(xyz=(0.0, -0.107, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"clamp_bolt_{int((z - 2.4) * 1000)}",
        )

    # Side arm and bracing welded to the clamp band.
    mast.visual(
        Cylinder(radius=0.035, length=0.850),
        origin=Origin(xyz=(0.475, 0.0, 2.550), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="side_arm",
    )
    mast.visual(
        Box((0.110, 0.130, 0.060)),
        origin=Origin(xyz=(0.080, 0.0, 2.550)),
        material=dark_steel,
        name="arm_saddle",
    )

    brace_dx = 0.340
    brace_dz = 0.135
    brace_len = math.hypot(brace_dx, brace_dz)
    brace_pitch = math.atan2(brace_dx, brace_dz)
    for iy, y in enumerate((-0.034, 0.034)):
        mast.visual(
            Cylinder(radius=0.014, length=brace_len),
            origin=Origin(
                xyz=(0.220, y, 2.4875),
                rpy=(0.0, brace_pitch, 0.0),
            ),
            material=galvanized,
            name=f"diagonal_brace_{iy}",
        )

    # Vertical bearing socket at the arm tip for the panning knuckle.
    mast.visual(
        Cylinder(radius=0.055, length=0.120),
        origin=Origin(xyz=(0.900, 0.0, 2.540)),
        material=dark_steel,
        name="end_socket",
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_steel,
        name="pan_cap",
    )
    pan_yoke.visual(
        Cylinder(radius=0.025, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=dark_steel,
        name="pan_stem",
    )
    pan_yoke.visual(
        Box((0.050, 0.210, 0.130)),
        origin=Origin(xyz=(-0.015, 0.0, -0.170)),
        material=dark_steel,
        name="yoke_bridge",
    )
    for iy, y in enumerate((-0.100, 0.100)):
        pan_yoke.visual(
            Box((0.200, 0.018, 0.140)),
            origin=Origin(xyz=(0.085, y, -0.180)),
            material=dark_steel,
            name=f"yoke_cheek_{iy}",
        )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.017, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_pin",
    )
    camera.visual(
        Cylinder(radius=0.055, length=0.345),
        origin=Origin(xyz=(0.1825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="body",
    )
    camera.visual(
        Cylinder(radius=0.057, length=0.024),
        origin=Origin(xyz=(0.354, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    camera.visual(
        Cylinder(radius=0.043, length=0.016),
        origin=Origin(xyz=(0.370, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="front_lens",
    )
    camera.visual(
        Box((0.390, 0.145, 0.014)),
        origin=Origin(xyz=(0.205, 0.0, 0.064)),
        material=dark_steel,
        name="sunshade_top",
    )
    for iy, y in enumerate((-0.067, 0.067)):
        camera.visual(
            Box((0.350, 0.012, 0.046)),
            origin=Origin(xyz=(0.210, y, 0.043)),
            material=dark_steel,
            name=f"sunshade_side_{iy}",
        )
    camera.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_gland",
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=pan_yoke,
        origin=Origin(xyz=(0.900, 0.0, 2.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera,
        origin=Origin(xyz=(0.100, 0.0, -0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.45, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    pan_yoke = object_model.get_part("pan_yoke")
    camera = object_model.get_part("camera")
    pan = object_model.get_articulation("pan_joint")
    tilt = object_model.get_articulation("tilt_joint")

    ctx.check(
        "camera uses pan and tilt revolute joints",
        pan.articulation_type == ArticulationType.REVOLUTE
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and pan.axis == (0.0, 0.0, 1.0)
        and tilt.axis == (0.0, 1.0, 0.0),
        details=f"pan={pan.articulation_type} axis={pan.axis}, tilt={tilt.articulation_type} axis={tilt.axis}",
    )
    ctx.check(
        "joint limits match camera knuckle ranges",
        pan.motion_limits is not None
        and tilt.motion_limits is not None
        and pan.motion_limits.lower <= -3.0
        and pan.motion_limits.upper >= 3.0
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.8,
        details=f"pan_limits={pan.motion_limits}, tilt_limits={tilt.motion_limits}",
    )

    ctx.expect_contact(
        pan_yoke,
        mast,
        elem_a="pan_cap",
        elem_b="end_socket",
        contact_tol=0.003,
        name="pan cap seats against arm-tip socket",
    )
    for cheek in ("yoke_cheek_0", "yoke_cheek_1"):
        ctx.expect_contact(
            camera,
            pan_yoke,
            elem_a="tilt_pin",
            elem_b=cheek,
            contact_tol=0.003,
            name=f"tilt pin reaches {cheek}",
        )
        ctx.expect_overlap(
            camera,
            pan_yoke,
            axes="xz",
            elem_a="tilt_pin",
            elem_b=cheek,
            min_overlap=0.010,
            name=f"tilt pin is aligned with {cheek}",
        )

    def _elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_body_center = _elem_center(camera, "body")
    with ctx.pose({pan: 0.75}):
        panned_body_center = _elem_center(camera, "body")
    ctx.check(
        "positive pan swings camera laterally around vertical mast axis",
        rest_body_center is not None
        and panned_body_center is not None
        and panned_body_center[1] > rest_body_center[1] + 0.12,
        details=f"rest={rest_body_center}, panned={panned_body_center}",
    )

    with ctx.pose({tilt: 0.90}):
        down_tilt_body_center = _elem_center(camera, "body")
    ctx.check(
        "positive tilt pitches bullet camera downward",
        rest_body_center is not None
        and down_tilt_body_center is not None
        and down_tilt_body_center[2] < rest_body_center[2] - 0.08,
        details=f"rest={rest_body_center}, tilted={down_tilt_body_center}",
    )

    return ctx.report()


object_model = build_object_model()
