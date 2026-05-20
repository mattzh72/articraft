from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _lamp_housing_shell():
    """Thin-walled spun/cast lamp body, authored along local +Z."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.142, -0.315),
            (0.168, -0.255),
            (0.192, -0.115),
            (0.212, 0.110),
            (0.226, 0.300),
            (0.215, 0.375),
        ],
        [
            (0.118, -0.290),
            (0.139, -0.235),
            (0.166, -0.105),
            (0.187, 0.110),
            (0.198, 0.300),
            (0.188, 0.355),
        ],
        segments=80,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _reflector_bowl():
    """Shiny concave reflector seated just inside the front aperture."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.045, 0.010),
            (0.088, 0.075),
            (0.137, 0.190),
            (0.188, 0.320),
        ],
        [
            (0.032, 0.018),
            (0.077, 0.080),
            (0.126, 0.192),
            (0.176, 0.312),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _top_handle():
    return tube_from_spline_points(
        [
            (-0.190, 0.0, 0.172),
            (-0.110, 0.0, 0.270),
            (0.085, 0.0, 0.292),
            (0.205, 0.0, 0.205),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _barn_leaf(part, *, orientation: str, material):
    """Add a hinged black barn-door leaf. Local origin is the hinge axis."""
    if orientation == "top":
        part.visual(
            Cylinder(radius=0.012, length=0.380),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name="hinge_barrel",
        )
        part.visual(
            Box((0.020, 0.360, 0.205)),
            origin=Origin(xyz=(0.034, 0.0, 0.108)),
            material=material,
            name="leaf_panel",
        )
        part.visual(
            Box((0.024, 0.360, 0.014)),
            origin=Origin(xyz=(0.043, 0.0, 0.214)),
            material=material,
            name="outer_lip",
        )
    elif orientation == "bottom":
        part.visual(
            Cylinder(radius=0.012, length=0.380),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name="hinge_barrel",
        )
        part.visual(
            Box((0.020, 0.360, 0.205)),
            origin=Origin(xyz=(0.034, 0.0, -0.108)),
            material=material,
            name="leaf_panel",
        )
        part.visual(
            Box((0.024, 0.360, 0.014)),
            origin=Origin(xyz=(0.043, 0.0, -0.214)),
            material=material,
            name="outer_lip",
        )
    elif orientation == "side_pos":
        part.visual(
            Cylinder(radius=0.012, length=0.360),
            origin=Origin(xyz=(0.018, 0.0, 0.0)),
            material=material,
            name="hinge_barrel",
        )
        part.visual(
            Box((0.020, 0.205, 0.340)),
            origin=Origin(xyz=(0.034, 0.108, 0.0)),
            material=material,
            name="leaf_panel",
        )
        part.visual(
            Box((0.024, 0.014, 0.340)),
            origin=Origin(xyz=(0.043, 0.214, 0.0)),
            material=material,
            name="outer_lip",
        )
    elif orientation == "side_neg":
        part.visual(
            Cylinder(radius=0.012, length=0.360),
            origin=Origin(xyz=(0.018, 0.0, 0.0)),
            material=material,
            name="hinge_barrel",
        )
        part.visual(
            Box((0.020, 0.205, 0.340)),
            origin=Origin(xyz=(0.034, -0.108, 0.0)),
            material=material,
            name="leaf_panel",
        )
        part.visual(
            Box((0.024, 0.014, 0.340)),
            origin=Origin(xyz=(0.043, -0.214, 0.0)),
            material=material,
            name="outer_lip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_yoke_spotlight")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.047, 0.050, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.15, 0.16, 0.17, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.95, 0.78, 0.34, 0.45))
    cool_glass = model.material("cool_glass", rgba=(0.56, 0.72, 0.92, 0.34))
    reflector = model.material("reflector", rgba=(0.92, 0.87, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.006, 0.006, 0.007, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.320, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_black,
        name="weighted_base",
    )
    stand.visual(
        _mesh(TorusGeometry(radius=0.292, tube=0.010, radial_segments=16, tubular_segments=72), "base_rubber_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=rubber,
        name="base_rubber_ring",
    )
    stand.visual(
        Cylinder(radius=0.040, length=1.160),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=dark_steel,
        name="lower_stand_tube",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 1.350)),
        material=brushed_steel,
        name="upper_stand_tube",
    )
    stand.visual(
        Cylinder(radius=0.060, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.185)),
        material=satin_black,
        name="height_lock_collar",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.105),
        origin=Origin(xyz=(0.075, 0.0, 1.185), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="height_lock_knob",
    )
    stand.visual(
        Cylinder(radius=0.066, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 1.530)),
        material=dark_steel,
        name="pan_receiver",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        stand.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(
                xyz=(0.225 * math.cos(angle), 0.225 * math.sin(angle), 0.066),
            ),
            material=brushed_steel,
            name=f"base_bolt_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        _mesh(
            TrunnionYokeGeometry(
                (0.720, 0.170, 0.545),
                span_width=0.540,
                trunnion_diameter=0.105,
                trunnion_center_z=0.382,
                base_thickness=0.062,
                corner_radius=0.018,
                center=False,
            ),
            "yoke_frame",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=satin_black,
        name="yoke_frame",
    )
    yoke.visual(
        Cylinder(radius=0.055, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="pan_spigot",
    )
    for index, (x, y) in enumerate(((-0.040, -0.165), (0.040, -0.165), (-0.040, 0.165), (0.040, 0.165))):
        yoke.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(x, y, 0.066)),
            material=brushed_steel,
            name=f"yoke_base_bolt_{index}",
        )

    lamp = model.part("lamp")
    lamp.visual(
        _mesh(_lamp_housing_shell(), "lamp_housing_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lamp_housing",
    )
    lamp.visual(
        _mesh(_reflector_bowl(), "reflector_bowl"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reflector,
        name="reflector_bowl",
    )
    lamp.visual(
        Cylinder(radius=0.188, length=0.018),
        origin=Origin(xyz=(0.346, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cool_glass,
        name="front_lens",
    )
    lamp.visual(
        _mesh(TorusGeometry(radius=0.193, tube=0.012, radial_segments=16, tubular_segments=80), "front_bezel"),
        origin=Origin(xyz=(0.372, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    for index, ring_radius in enumerate((0.055, 0.088, 0.122, 0.154)):
        lamp.visual(
            _mesh(TorusGeometry(radius=ring_radius, tube=0.0025, radial_segments=8, tubular_segments=48), f"fresnel_ring_{index}"),
            origin=Origin(xyz=(0.354, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_glass,
            name=f"fresnel_ring_{index}",
        )
    lamp.visual(
        _mesh(
            VentGrilleGeometry(
                (0.220, 0.220),
                frame=0.018,
                face_thickness=0.006,
                duct_depth=0.020,
                slat_pitch=0.024,
                slat_width=0.010,
                slat_angle_deg=28.0,
                corner_radius=0.018,
                slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
                mounts=VentGrilleMounts(style="holes", inset=0.020, hole_diameter=0.005),
            ),
            "rear_vent_grille",
        ),
        origin=Origin(xyz=(-0.316, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_vent_grille",
    )
    lamp.visual(
        Cylinder(radius=0.0525, length=0.820),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_shaft",
    )
    for side, y_pos in (("pos", 0.198), ("neg", -0.198)):
        lamp.visual(
            Cylinder(radius=0.073, length=0.034),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"trunnion_boss_{side}",
        )
    for side, y_pos in (("pos", 0.390), ("neg", -0.390)):
        lamp.visual(
            _mesh(
                KnobGeometry(
                    0.110,
                    0.050,
                    body_style="lobed",
                    top_diameter=0.098,
                    grip=KnobGrip(style="ribbed", count=12, depth=0.0020),
                    bore=KnobBore(style="round", diameter=0.018),
                ),
                f"tilt_lock_knob_{side}",
            ),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name=f"tilt_lock_knob_{side}",
        )
    lamp.visual(
        _mesh(_top_handle(), "top_handle"),
        material=satin_black,
        name="top_handle",
    )
    for index, x_pos in enumerate((-0.190, 0.205)):
        lamp.visual(
            Cylinder(radius=0.026, length=0.024),
            origin=Origin(xyz=(x_pos, 0.0, 0.188)),
            material=satin_black,
            name=f"handle_mount_{index}",
        )
    lamp.visual(
        Box((0.110, 0.010, 0.052)),
        origin=Origin(xyz=(0.105, 0.216, -0.030)),
        material=brushed_steel,
        name="side_data_plate",
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="knob_stem",
    )
    focus_knob.visual(
        _mesh(
            KnobGeometry(
                0.052,
                0.034,
                body_style="faceted",
                top_diameter=0.044,
                grip=KnobGrip(style="knurled", count=28, depth=0.0012, helix_angle_deg=18.0),
                bore=KnobBore(style="round", diameter=0.010),
            ),
            "focus_knob",
        ),
        origin=Origin(xyz=(-0.032, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knob_cap",
    )

    top_leaf = model.part("top_leaf")
    _barn_leaf(top_leaf, orientation="top", material=satin_black)
    bottom_leaf = model.part("bottom_leaf")
    _barn_leaf(bottom_leaf, orientation="bottom", material=satin_black)
    side_leaf_0 = model.part("side_leaf_0")
    _barn_leaf(side_leaf_0, orientation="side_pos", material=satin_black)
    side_leaf_1 = model.part("side_leaf_1")
    _barn_leaf(side_leaf_1, orientation="side_neg", material=satin_black)

    model.articulation(
        "stand_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
    )
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.382)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.9, lower=math.radians(-55.0), upper=math.radians(65.0)),
    )
    model.articulation(
        "lamp_to_focus_knob",
        ArticulationType.CONTINUOUS,
        parent=lamp,
        child=focus_knob,
        origin=Origin(xyz=(-0.326, -0.095, 0.090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )
    model.articulation(
        "lamp_to_top_leaf",
        ArticulationType.REVOLUTE,
        parent=lamp,
        child=top_leaf,
        origin=Origin(xyz=(0.378, 0.0, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=math.radians(-70.0), upper=math.radians(35.0)),
    )
    model.articulation(
        "lamp_to_bottom_leaf",
        ArticulationType.REVOLUTE,
        parent=lamp,
        child=bottom_leaf,
        origin=Origin(xyz=(0.378, 0.0, -0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=math.radians(-35.0), upper=math.radians(70.0)),
    )
    model.articulation(
        "lamp_to_side_leaf_0",
        ArticulationType.REVOLUTE,
        parent=lamp,
        child=side_leaf_0,
        origin=Origin(xyz=(0.378, 0.214, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=math.radians(-35.0), upper=math.radians(70.0)),
    )
    model.articulation(
        "lamp_to_side_leaf_1",
        ArticulationType.REVOLUTE,
        parent=lamp,
        child=side_leaf_1,
        origin=Origin(xyz=(0.378, -0.214, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=math.radians(-70.0), upper=math.radians(35.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    focus_knob = object_model.get_part("focus_knob")
    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_lamp")
    focus = object_model.get_articulation("lamp_to_focus_knob")

    ctx.allow_overlap(
        stand,
        yoke,
        elem_a="pan_receiver",
        elem_b="pan_spigot",
        reason="The pan spigot is intentionally seated inside the stand receiver socket.",
    )
    ctx.allow_overlap(
        lamp,
        yoke,
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        reason="The trunnion shaft is intentionally captured in the yoke bore for the tilt pivot.",
    )
    ctx.allow_overlap(
        focus_knob,
        lamp,
        elem_a="knob_stem",
        elem_b="lamp_housing",
        reason="The small focus control stem is intentionally inserted through the lamp housing.",
    )
    ctx.allow_overlap(
        focus_knob,
        lamp,
        elem_a="knob_stem",
        elem_b="rear_vent_grille",
        reason="The focus control stem passes through the rear service grille into the focusing mechanism.",
    )

    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        positive_elem="pan_spigot",
        negative_elem="pan_receiver",
        max_gap=0.001,
        max_penetration=0.025,
        name="pan spigot remains seated in receiver",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="yoke_frame",
        margin=0.010,
        name="trunnion shaft is centered in yoke bore",
    )
    ctx.expect_overlap(
        lamp,
        yoke,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        min_overlap=0.60,
        name="trunnion shaft spans both yoke cheeks",
    )
    ctx.expect_overlap(
        focus_knob,
        lamp,
        axes="yz",
        elem_a="knob_stem",
        elem_b="lamp_housing",
        min_overlap=0.010,
        name="focus stem footprint is seated in housing",
    )
    ctx.expect_overlap(
        focus_knob,
        lamp,
        axes="yz",
        elem_a="knob_stem",
        elem_b="rear_vent_grille",
        min_overlap=0.010,
        name="focus stem passes through rear panel",
    )
    ctx.check(
        "focus knob is a rotary control",
        focus.articulation_type == ArticulationType.CONTINUOUS,
        details=f"focus articulation type is {focus.articulation_type}",
    )

    rest_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({tilt: math.radians(45.0)}):
        raised_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    ctx.check(
        "positive tilt raises beam",
        rest_aabb is not None
        and raised_aabb is not None
        and (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5 > (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    rest_front = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({pan: math.radians(90.0)}):
        panned_front = ctx.part_element_world_aabb(lamp, elem="front_lens")
    ctx.check(
        "pan rotates the beam about the stand",
        rest_front is not None
        and panned_front is not None
        and abs(((panned_front[0][1] + panned_front[1][1]) * 0.5) - ((rest_front[0][1] + rest_front[1][1]) * 0.5)) > 0.25,
        details=f"rest={rest_front}, panned={panned_front}",
    )

    return ctx.report()


object_model = build_object_model()
