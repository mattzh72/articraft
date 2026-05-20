from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


PAN_X = 0.54
ARM_Z = 0.45
PAN_Z = 0.405
TILT_Z = -0.125


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_mount_cctv_arm")

    galvanized = model.material("galvanized_steel", rgba=(0.63, 0.66, 0.64, 1.0))
    dark_galv = model.material("dark_galvanized_edges", rgba=(0.37, 0.39, 0.38, 1.0))
    screw_black = model.material("blackened_screws", rgba=(0.06, 0.065, 0.07, 1.0))
    camera_white = model.material("powder_coated_white", rgba=(0.92, 0.93, 0.90, 1.0))
    camera_black = model.material("smoked_black_dome", rgba=(0.015, 0.020, 0.025, 0.70))

    mount = model.part("mount_arm")

    # A broad tapered sheet is the distinctive diagonal galvanized corner plate:
    # its base is high on the corner, while the top and bottom edges slope into
    # a short flat tip where the arm tube is welded.
    diagonal_profile = [
        (-0.025, -0.170),
        (-0.025, 0.170),
        (0.230, 0.030),
        (0.230, -0.030),
    ]
    diagonal_plate = ExtrudeGeometry(diagonal_profile, 0.012, center=True).rotate_x(math.pi / 2)
    diagonal_plate.translate(0.0, 0.0, ARM_Z)
    mount.visual(
        mesh_from_geometry(diagonal_plate, "diagonal_plate"),
        material=galvanized,
        name="diagonal_plate",
    )

    # The two short leaves behind the diagonal sheet suggest a real corner
    # bracket bolted to perpendicular walls.
    for index, (yaw, y) in enumerate(((math.radians(42.0), 0.052), (-math.radians(42.0), -0.052))):
        mount.visual(
            Box((0.012, 0.105, 0.380)),
            origin=Origin(xyz=(-0.028, y, ARM_Z), rpy=(0.0, 0.0, yaw)),
            material=galvanized,
            name=f"corner_leaf_{index}",
        )
    mount.visual(
        Box((0.030, 0.120, 0.380)),
        origin=Origin(xyz=(-0.032, 0.0, ARM_Z)),
        material=dark_galv,
        name="corner_spine",
    )

    # Bolt heads are seated into the diagonal plate rather than floating on it.
    bolt_points = [
        (0.020, 0.0, ARM_Z - 0.105),
        (0.020, 0.0, ARM_Z + 0.105),
        (0.130, 0.0, ARM_Z - 0.045),
        (0.130, 0.0, ARM_Z + 0.045),
    ]
    for index, xyz in enumerate(bolt_points):
        mount.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(xyz[0], -0.0085, xyz[2]), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=screw_black,
            name=f"plate_bolt_{index}",
        )

    # The short horizontal galvanized arm projects from the tip of the plate.
    mount.visual(
        Cylinder(radius=0.022, length=0.315),
        origin=Origin(xyz=(0.382, 0.0, ARM_Z), rpy=(0.0, math.pi / 2, 0.0)),
        material=galvanized,
        name="arm_tube",
    )
    mount.visual(
        Cylinder(radius=0.035, length=0.034),
        origin=Origin(xyz=(0.235, 0.0, ARM_Z), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_galv,
        name="weld_collar",
    )
    mount.visual(
        Box((0.170, 0.008, 0.050)),
        origin=Origin(xyz=(0.315, 0.0, ARM_Z - 0.032), rpy=(0.0, 0.0, 0.0)),
        material=dark_galv,
        name="under_gusset",
    )

    # A static lower bearing at the arm end carries the pan axis.  The rotating
    # yoke sits directly below this face.
    mount.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(PAN_X, 0.0, PAN_Z + 0.030)),
        material=galvanized,
        name="pan_bearing",
    )
    mount.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.033, tube=0.004, radial_segments=16, tubular_segments=48),
            "bearing_lip",
        ),
        origin=Origin(xyz=(PAN_X, 0.0, PAN_Z + 0.061)),
        material=dark_galv,
        name="bearing_lip",
    )

    yoke = model.part("pan_yoke")
    yoke.visual(
        Cylinder(radius=0.016, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=camera_white,
        name="pan_stem",
    )
    yoke.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=camera_white,
        name="turntable",
    )
    yoke.visual(
        Box((0.080, 0.145, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=camera_white,
        name="yoke_crown",
    )
    yoke.visual(
        Box((0.030, 0.012, 0.135)),
        origin=Origin(xyz=(0.0, -0.066, -0.125)),
        material=camera_white,
        name="side_cheek_0",
    )
    yoke.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, -0.075, TILT_Z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=screw_black,
        name="pivot_cap_0",
    )
    yoke.visual(
        Box((0.030, 0.012, 0.135)),
        origin=Origin(xyz=(0.0, 0.066, -0.125)),
        material=camera_white,
        name="side_cheek_1",
    )
    yoke.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.075, TILT_Z), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=screw_black,
        name="pivot_cap_1",
    )

    camera = model.part("dome_camera")
    camera.visual(
        Cylinder(radius=0.0088, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=dark_galv,
        name="tilt_pin",
    )
    camera.visual(
        Cylinder(radius=0.030, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=camera_white,
        name="tilt_barrel",
    )
    camera.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=camera_white,
        name="housing_band",
    )
    camera.visual(
        mesh_from_geometry(DomeGeometry(0.045, radial_segments=48, height_segments=14), "tinted_dome"),
        origin=Origin(xyz=(0.0, 0.0, -0.0525), rpy=(math.pi, 0.0, 0.0)),
        material=camera_black,
        name="tinted_dome",
    )
    camera.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(xyz=(0.020, 0.0, -0.090), rpy=(0.0, math.pi / 2, 0.0)),
        material=screw_black,
        name="lens_window",
    )

    model.articulation(
        "arm_to_yoke",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=yoke,
        origin=Origin(xyz=(PAN_X, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "yoke_to_camera",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, TILT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount_arm")
    yoke = object_model.get_part("pan_yoke")
    camera = object_model.get_part("dome_camera")
    pan = object_model.get_articulation("arm_to_yoke")
    tilt = object_model.get_articulation("yoke_to_camera")

    ctx.expect_gap(
        mount,
        yoke,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pan_bearing",
        negative_elem="turntable",
        name="turntable seats under arm bearing",
    )
    ctx.expect_contact(
        camera,
        yoke,
        elem_a="tilt_pin",
        elem_b="side_cheek_0",
        contact_tol=0.00001,
        name="tilt pin reaches first yoke cheek",
    )
    ctx.expect_contact(
        camera,
        yoke,
        elem_a="tilt_pin",
        elem_b="side_cheek_1",
        contact_tol=0.00001,
        name="tilt pin reaches second yoke cheek",
    )

    pan_limits = getattr(pan, "motion_limits", None)
    tilt_limits = getattr(tilt, "motion_limits", None)
    ctx.check(
        "pan joint has broad stopped rotation",
        pan_limits is not None and pan_limits.lower < -2.0 and pan_limits.upper > 2.0,
        details=f"pan_limits={pan_limits}",
    )
    ctx.check(
        "tilt joint has realistic camera pitch range",
        tilt_limits is not None and -1.0 < tilt_limits.lower < -0.5 and 0.5 < tilt_limits.upper < 1.0,
        details=f"tilt_limits={tilt_limits}",
    )

    def _center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (float(aabb[0][0]) + float(aabb[1][0]))

    rest_dome = ctx.part_element_world_aabb(camera, elem="tinted_dome")
    rest_x = _center_x(rest_dome)
    with ctx.pose({tilt: 0.65}):
        tilted_dome = ctx.part_element_world_aabb(camera, elem="tinted_dome")
        tilted_x = _center_x(tilted_dome)
    ctx.check(
        "tilt joint pitches dome on horizontal axis",
        rest_x is not None and tilted_x is not None and tilted_x < rest_x - 0.020,
        details=f"rest_x={rest_x}, tilted_x={tilted_x}",
    )

    return ctx.report()


object_model = build_object_model()
