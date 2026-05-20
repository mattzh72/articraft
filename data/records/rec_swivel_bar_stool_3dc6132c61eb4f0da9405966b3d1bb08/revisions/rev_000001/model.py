from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.20, 0.12, 0.08, 1.0))
    seat_pan_black = model.material("seat_pan_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base_plate_mesh = _mesh(
        "bar_stool_base_plate",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.080, 0.0),
                (0.168, 0.004),
                (0.212, 0.011),
                (0.230, 0.021),
                (0.226, 0.031),
                (0.194, 0.037),
                (0.104, 0.039),
                (0.0, 0.039),
            ],
            segments=72,
        ),
    )
    seat_cushion_mesh = _mesh(
        "bar_stool_seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.082, 0.0),
                (0.134, 0.004),
                (0.170, 0.014),
                (0.186, 0.028),
                (0.188, 0.046),
                (0.162, 0.060),
                (0.088, 0.067),
                (0.0, 0.065),
            ],
            segments=72,
        ),
    )
    foot_ring_mesh = _mesh(
        "bar_stool_foot_ring",
        TorusGeometry(radius=0.155, tube=0.012, radial_segments=18, tubular_segments=72),
    )

    base = model.part("base")
    base.visual(base_plate_mesh, material=powder_black, name="floor_base")
    base.visual(
        Cylinder(radius=0.032, length=0.626),
        origin=Origin(xyz=(0.0, 0.0, 0.352)),
        material=satin_steel,
        name="pedestal_post",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_trim,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.651)),
        material=dark_trim,
        name="upper_post_collar",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.460, 0.460, 0.665)),
        mass=17.5,
        origin=Origin(xyz=(0.0, 0.0, 0.3325)),
    )

    foot_ring = model.part("foot_ring")
    foot_ring.visual(foot_ring_mesh, material=satin_steel, name="ring")
    foot_ring.visual(
        Box((0.018, 0.030, 0.020)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=dark_trim,
        name="clamp_pad_pos_x",
    )
    foot_ring.visual(
        Box((0.018, 0.030, 0.020)),
        origin=Origin(xyz=(-0.041, 0.0, 0.0)),
        material=dark_trim,
        name="clamp_pad_neg_x",
    )
    foot_ring.visual(
        Box((0.030, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.041, 0.0)),
        material=dark_trim,
        name="clamp_pad_pos_y",
    )
    foot_ring.visual(
        Box((0.030, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.041, 0.0)),
        material=dark_trim,
        name="clamp_pad_neg_y",
    )
    foot_ring.visual(
        Cylinder(radius=0.007, length=0.093),
        origin=Origin(xyz=(0.0965, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="spoke_pos_x",
    )
    foot_ring.visual(
        Cylinder(radius=0.007, length=0.093),
        origin=Origin(xyz=(-0.0965, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="spoke_neg_x",
    )
    foot_ring.visual(
        Cylinder(radius=0.007, length=0.093),
        origin=Origin(xyz=(0.0, 0.0965, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="spoke_pos_y",
    )
    foot_ring.visual(
        Cylinder(radius=0.007, length=0.093),
        origin=Origin(xyz=(0.0, -0.0965, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="spoke_neg_y",
    )
    foot_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.167, length=0.040),
        mass=1.7,
        origin=Origin(),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_trim,
        name="lower_flange",
    )
    bearing_module.visual(
        Cylinder(radius=0.047, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_steel,
        name="bearing_drum",
    )
    bearing_module.visual(
        Cylinder(radius=0.055, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0455)),
        material=dark_trim,
        name="upper_cap",
    )
    bearing_module.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.055),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    head_module = model.part("head_module")
    head_module.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_steel,
        name="swivel_spindle",
    )
    head_module.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_trim,
        name="top_boss",
    )
    head_module.visual(
        Box((0.170, 0.170, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_trim,
        name="seat_plate",
    )
    head_module.visual(
        Box((0.150, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_trim,
        name="brace_x",
    )
    head_module.visual(
        Box((0.036, 0.150, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_trim,
        name="brace_y",
    )
    head_module.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.042)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.162, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=seat_pan_black,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_trim,
        name="mount_pad",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.188, length=0.077),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0385)),
    )

    model.articulation(
        "base_to_foot_ring",
        ArticulationType.FIXED,
        parent=base,
        child=foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
    )
    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
    )
    model.articulation(
        "bearing_to_head_swivel",
        ArticulationType.CONTINUOUS,
        parent=bearing_module,
        child=head_module,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )
    model.articulation(
        "head_to_seat",
        ArticulationType.FIXED,
        parent=head_module,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    foot_ring = object_model.get_part("foot_ring")
    bearing_module = object_model.get_part("bearing_module")
    head_module = object_model.get_part("head_module")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("bearing_to_head_swivel")

    ctx.check(
        "seat swivel is a vertical continuous joint",
        swivel.articulation_type == ArticulationType.CONTINUOUS and swivel.axis == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )

    ctx.expect_gap(
        bearing_module,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="bearing module seats on pedestal top",
    )
    ctx.expect_gap(
        head_module,
        bearing_module,
        axis="z",
        min_gap=0.020,
        max_gap=0.028,
        positive_elem="top_boss",
        negative_elem="upper_cap",
        name="head module stands proud above the bearing housing",
    )
    ctx.expect_gap(
        seat,
        head_module,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat pan sits directly on the head module",
    )
    ctx.expect_origin_gap(
        seat,
        base,
        axis="z",
        min_gap=0.75,
        max_gap=0.81,
        name="seat height reads as a bar stool",
    )
    ctx.expect_origin_gap(
        foot_ring,
        base,
        axis="z",
        min_gap=0.30,
        max_gap=0.34,
        name="foot ring is mounted midway up the pedestal",
    )
    ctx.expect_origin_distance(
        foot_ring,
        base,
        axes="xy",
        max_dist=0.001,
        name="foot ring stays centered on the pedestal axis",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({swivel: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(seat)
        ctx.expect_gap(
            head_module,
            bearing_module,
            axis="z",
            min_gap=0.020,
            max_gap=0.028,
            positive_elem="top_boss",
            negative_elem="upper_cap",
            name="head stand-off stays clean when the seat rotates",
        )

    ctx.check(
        "seat rotates in place around the pedestal axis",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rest_pos[0]) < 1e-6
        and abs(rest_pos[1]) < 1e-6
        and abs(rotated_pos[0] - rest_pos[0]) < 1e-6
        and abs(rotated_pos[1] - rest_pos[1]) < 1e-6
        and abs(rotated_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
