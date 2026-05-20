from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HINGE_AXIS_Z = 0.020
LEFT_Y = 0.067
RIGHT_Y = -0.067
BODY_LENGTH = 0.250


def _shell_ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 48,
):
    """A short hollow sleeve whose local axis is +Z before visual placement."""
    half = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def _ribbed_sleeve_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    rib_count: int,
):
    """Hollow eyepiece/diopter sleeve with raised rubber grip ribs."""
    half = length * 0.5
    rib_step = length / (rib_count * 2.0 + 1.0)
    outer_profile: list[tuple[float, float]] = [(-half, outer_radius)]
    z = -half
    for _ in range(rib_count):
        z += rib_step
        outer_profile.append((z, outer_radius))
        outer_profile.append((z, outer_radius + 0.0020))
        z += rib_step
        outer_profile.append((z, outer_radius + 0.0020))
        outer_profile.append((z, outer_radius))
    outer_profile.append((half, outer_radius))
    geom = LatheGeometry.from_shell_profiles(
        [(r, z) for z, r in outer_profile],
        [(inner_radius, -half), (inner_radius, half)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def _add_barrel_side(
    part,
    *,
    side_y: float,
    rubber: Material,
    black: Material,
    glass: Material,
    trim: Material,
    name_prefix: str,
) -> None:
    """Build one porro-prism optical half in the part's local hinge frame."""
    sign = 1.0 if side_y > 0.0 else -1.0
    prism_y = side_y * 0.92
    eyepiece_y = side_y * 0.78
    eyepiece_tube_name = "right_eyepiece_tube" if name_prefix == "right" else "left_eyepiece_tube"

    # Long rubber objective tube and raised grip collars.
    part.visual(
        Cylinder(radius=0.032, length=0.160),
        origin=Origin(xyz=(0.035, side_y, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name=f"{name_prefix}_tube",
    )
    for index, x_pos in enumerate((-0.018, 0.028, 0.074)):
        part.visual(
            Cylinder(radius=0.0345, length=0.010),
            origin=Origin(xyz=(x_pos, side_y, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"{name_prefix}_armor_band_{index}",
        )

    # Larger front objective bell with glass recessed just behind a black rim.
    part.visual(
        Cylinder(radius=0.043, length=0.048),
        origin=Origin(xyz=(0.124, side_y, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name=f"{name_prefix}_objective_bell",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.151, side_y, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name=f"{name_prefix}_objective_rim",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.154, side_y, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name=f"{name_prefix}_objective_glass",
    )

    # The blocky shoulders make the silhouette read as a porro-prism binocular.
    part.visual(
        Box((0.090, 0.052, 0.060)),
        origin=Origin(xyz=(-0.020, prism_y, 0.037)),
        material=rubber,
        name=f"{name_prefix}_prism_housing",
    )
    part.visual(
        Box((0.058, 0.020, 0.012)),
        origin=Origin(xyz=(-0.018, side_y - sign * 0.027, 0.066)),
        material=black,
        name=f"{name_prefix}_top_grip_pad",
    )
    part.visual(
        Box((0.052, 0.012, 0.045)),
        origin=Origin(xyz=(-0.012, side_y - sign * 0.047, 0.034)),
        material=black,
        name=f"{name_prefix}_side_grip_pad",
    )

    # Rear eyepiece line is slightly inboard and higher, as on porro optics.
    part.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(xyz=(-0.095, eyepiece_y, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name=eyepiece_tube_name,
    )
    part.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(-0.142, eyepiece_y, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name=f"{name_prefix}_eyecup",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(-0.157, eyepiece_y, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name=f"{name_prefix}_ocular_glass",
    )

    # Solid rubber bridge from hinge spine into the optical/prism housing.
    part.visual(
        Box((0.072, 0.050, 0.018)),
        origin=Origin(xyz=(-0.010, side_y * 0.59, HINGE_AXIS_Z + 0.004)),
        material=rubber,
        name=f"{name_prefix}_lower_bridge",
    )
    part.visual(
        Box((0.052, 0.032, 0.016)),
        origin=Origin(xyz=(-0.075, side_y * 0.70, 0.052)),
        material=rubber,
        name=f"{name_prefix}_focus_bridge",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_7x50_porro_binocular")

    rubber = model.material("pebbled_navy_rubber", rgba=(0.02, 0.055, 0.060, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.006, 0.007, 0.008, 1.0))
    glass = model.material("blue_green_coated_glass", rgba=(0.20, 0.48, 0.55, 0.50))
    trim = model.material("anodized_black_trim", rgba=(0.015, 0.017, 0.018, 1.0))
    metal = model.material("dark_stainless_pin", rgba=(0.25, 0.27, 0.28, 1.0))
    marking = model.material("white_engraving", rgba=(0.92, 0.95, 0.90, 1.0))

    hinge_sleeve_short = _shell_ring_mesh(
        "hinge_sleeve_short",
        outer_radius=0.014,
        inner_radius=0.0075,
        length=0.046,
    )
    hinge_sleeve_long = _shell_ring_mesh(
        "hinge_sleeve_long",
        outer_radius=0.014,
        inner_radius=0.0075,
        length=0.070,
    )
    focus_bearing_mesh = _shell_ring_mesh(
        "focus_bearing",
        outer_radius=0.011,
        inner_radius=0.0065,
        length=0.012,
    )
    diopter_mesh = _ribbed_sleeve_mesh(
        "ribbed_diopter_ring",
        outer_radius=0.030,
        inner_radius=0.0235,
        length=0.022,
        rib_count=5,
    )

    left_barrel = model.part("left_barrel")
    _add_barrel_side(
        left_barrel,
        side_y=LEFT_Y,
        rubber=rubber,
        black=black,
        glass=glass,
        trim=trim,
        name_prefix="left",
    )
    # Central hinge pin and fixed knuckle portions.  These are part of the left
    # half, giving the binocular a single root frame while the other half swings.
    left_barrel.visual(
        Cylinder(radius=0.006, length=BODY_LENGTH),
        origin=Origin(xyz=(0.015, 0.0, HINGE_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    left_barrel.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.113, 0.0, HINGE_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin_cap",
    )
    for index, x_pos in enumerate((-0.088, 0.083)):
        left_barrel.visual(
            hinge_sleeve_short,
            origin=Origin(xyz=(x_pos, 0.0, HINGE_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"left_hinge_knuckle_{index}",
        )
    left_barrel.visual(
        Box((0.225, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.0185, HINGE_AXIS_Z)),
        material=rubber,
        name="left_hinge_web",
    )
    left_barrel.visual(
        focus_bearing_mesh,
        origin=Origin(xyz=(-0.070, 0.026, 0.059), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="focus_bearing_0",
    )
    left_barrel.visual(
        Box((0.040, 0.072, 0.010)),
        origin=Origin(xyz=(-0.070, 0.0, 0.034)),
        material=rubber,
        name="focus_saddle",
    )
    left_barrel.visual(
        Box((0.040, 0.008, 0.002)),
        origin=Origin(xyz=(-0.020, 0.061, 0.068)),
        material=marking,
        name="seven_by_fifty_mark",
    )
    left_barrel.inertial = Inertial.from_geometry(
        Box((0.290, 0.170, 0.105)),
        mass=0.72,
        origin=Origin(xyz=(0.010, 0.035, 0.036)),
    )

    right_barrel = model.part("right_barrel")
    _add_barrel_side(
        right_barrel,
        side_y=RIGHT_Y,
        rubber=rubber,
        black=black,
        glass=glass,
        trim=trim,
        name_prefix="right",
    )
    right_barrel.visual(
        hinge_sleeve_long,
        origin=Origin(xyz=(0.000, 0.0, HINGE_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_hinge_knuckle",
    )
    right_barrel.visual(
        Box((0.072, 0.028, 0.018)),
        origin=Origin(xyz=(0.000, -0.023, HINGE_AXIS_Z + 0.004)),
        material=rubber,
        name="right_hinge_web",
    )
    right_barrel.inertial = Inertial.from_geometry(
        Box((0.290, 0.170, 0.105)),
        mass=0.70,
        origin=Origin(xyz=(0.010, -0.035, 0.036)),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.024,
                body_style="hourglass",
                grip=KnobGrip(style="ribbed", count=28, depth=0.0012),
            ),
            "center_focus_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="focus_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.0067, length=0.054),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="focus_axle",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.024),
        mass=0.045,
    )

    diopter_ring = model.part("diopter_ring")
    diopter_ring.visual(
        diopter_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="diopter_sleeve",
    )
    diopter_ring.visual(
        Box((0.010, 0.002, 0.003)),
        origin=Origin(xyz=(0.006, -0.0305, 0.0)),
        material=marking,
        name="diopter_index_mark",
    )
    diopter_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.022),
        mass=0.030,
    )

    model.articulation(
        "central_hinge",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child=right_barrel,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.8, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "center_focus",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child=focus_knob,
        origin=Origin(xyz=(-0.070, 0.0, 0.059)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.30, velocity=2.5, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "diopter_adjust",
        ArticulationType.REVOLUTE,
        parent=right_barrel,
        child=diopter_ring,
        origin=Origin(xyz=(-0.118, RIGHT_Y * 0.78, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.18, velocity=1.8, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_knob = object_model.get_part("focus_knob")
    diopter_ring = object_model.get_part("diopter_ring")
    hinge = object_model.get_articulation("central_hinge")
    focus = object_model.get_articulation("center_focus")
    diopter = object_model.get_articulation("diopter_adjust")

    ctx.expect_origin_gap(
        left_barrel,
        right_barrel,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        name="barrel frames share the central hinge line",
    )
    ctx.expect_overlap(
        left_barrel,
        right_barrel,
        axes="x",
        elem_a="hinge_pin",
        elem_b="right_hinge_knuckle",
        min_overlap=0.060,
        name="right hinge knuckle is retained on the central pin",
    )
    ctx.expect_within(
        focus_knob,
        left_barrel,
        axes="y",
        inner_elem="focus_axle",
        outer_elem="focus_saddle",
        margin=0.010,
        name="focus axle is captured by the central saddle",
    )
    ctx.allow_overlap(
        focus_knob,
        left_barrel,
        elem_a="focus_axle",
        elem_b="focus_bearing_0",
        reason="The metal focus shaft is intentionally seated through the rubber bearing sleeve so the knob is mechanically captured.",
    )
    ctx.expect_gap(
        left_barrel,
        focus_knob,
        axis="y",
        positive_elem="focus_bearing_0",
        negative_elem="focus_axle",
        max_penetration=0.008,
        name="focus shaft has only local bearing penetration",
    )
    ctx.expect_overlap(
        focus_knob,
        left_barrel,
        axes="y",
        elem_a="focus_axle",
        elem_b="focus_bearing_0",
        min_overlap=0.004,
        name="focus shaft remains retained in the bearing",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_barrel,
        axes="x",
        elem_a="diopter_sleeve",
        elem_b="right_eyepiece_tube",
        min_overlap=0.010,
        name="diopter ring sits on the right eyepiece",
    )

    rest_aabb = ctx.part_world_aabb(right_barrel)
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        swung_aabb = ctx.part_world_aabb(right_barrel)
    ctx.check(
        "central hinge changes the right barrel angle",
        rest_aabb is not None
        and swung_aabb is not None
        and abs(((swung_aabb[0][2] + swung_aabb[1][2]) - (rest_aabb[0][2] + rest_aabb[1][2])) * 0.5)
        > 0.004,
        details=f"rest={rest_aabb}, swung={swung_aabb}",
    )

    ctx.check(
        "focus and diopter are independent rotary controls",
        focus.motion_limits.lower < -2.0
        and focus.motion_limits.upper > 2.0
        and diopter.motion_limits.lower < -0.8
        and diopter.motion_limits.upper > 0.8,
        details=f"focus={focus.motion_limits}, diopter={diopter.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
