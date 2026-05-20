from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="y_fork_toddler_scooter")

    deck_body = model.material("deck_body", rgba=(0.33, 0.77, 0.76, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    alloy = model.material("alloy", rgba=(0.77, 0.79, 0.82, 1.0))
    rim_grey = model.material("rim_grey", rgba=(0.78, 0.80, 0.84, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.46, 0.50, 0.55, 1.0))
    accent = model.material("accent", rgba=(0.95, 0.52, 0.20, 1.0))

    stem_rake = radians(19.0)
    sleeve_start = (0.012, 0.0, 0.025)
    sleeve_length = 0.34
    sleeve_top = (
        sleeve_start[0] - sin(stem_rake) * sleeve_length,
        0.0,
        sleeve_start[2] + cos(stem_rake) * sleeve_length,
    )

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def wheel_tire_mesh(name: str, *, radius: float, width: float):
        crown_width = width * 0.94
        sidewall_width = width * 0.56
        bead_width = width * 0.20
        tire_geom = CylinderGeometry(radius, crown_width, radial_segments=56).rotate_x(-pi / 2.0)
        tire_geom.merge(
            CylinderGeometry(radius * 0.93, sidewall_width, radial_segments=56)
            .rotate_x(-pi / 2.0)
            .translate(0.0, -width * 0.22, 0.0)
        )
        tire_geom.merge(
            CylinderGeometry(radius * 0.93, sidewall_width, radial_segments=56)
            .rotate_x(-pi / 2.0)
            .translate(0.0, width * 0.22, 0.0)
        )
        tire_geom.merge(
            CylinderGeometry(radius * 0.74, bead_width, radial_segments=56)
            .rotate_x(-pi / 2.0)
            .translate(0.0, -width * 0.36, 0.0)
        )
        tire_geom.merge(
            CylinderGeometry(radius * 0.74, bead_width, radial_segments=56)
            .rotate_x(-pi / 2.0)
            .translate(0.0, width * 0.36, 0.0)
        )
        return save_mesh(name, tire_geom)

    def add_wheel(
        part_name: str,
        *,
        radius: float,
        width: float,
        mass: float,
        mesh_prefix: str,
    ):
        wheel = model.part(part_name)
        shoulder_length = width * 0.24
        shoulder_offset = width * 0.5 - shoulder_length * 0.5
        wheel.visual(
            wheel_tire_mesh(f"{mesh_prefix}_tire", radius=radius, width=width),
            material=dark_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.62, length=width * 0.22),
            origin=Origin(xyz=(0.0, -width * 0.27, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rim_grey,
            name="left_rim",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.62, length=width * 0.22),
            origin=Origin(xyz=(0.0, width * 0.27, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rim_grey,
            name="right_rim",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.22, length=width * 0.56),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hub_grey,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.12, length=width * 0.64),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=alloy,
            name="bearing_core",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.14, length=shoulder_length),
            origin=Origin(xyz=(0.0, -shoulder_offset, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=alloy,
            name="left_bearing_shoulder",
        )
        wheel.visual(
            Cylinder(radius=radius * 0.14, length=shoulder_length),
            origin=Origin(xyz=(0.0, shoulder_offset, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=alloy,
            name="right_bearing_shoulder",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=width),
            mass=mass,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )
        return wheel

    deck = model.part("deck_chassis")
    deck_geom = ExtrudeGeometry(
        rounded_rect_profile(0.54, 0.14, 0.028, corner_segments=8),
        0.03,
        center=True,
    ).translate(-0.02, 0.0, 0.05)
    deck_geom.merge(BoxGeometry((0.11, 0.100, 0.040)).translate(0.15, 0.0, 0.060))
    deck_geom.merge(BoxGeometry((0.085, 0.014, 0.100)).translate(0.150, 0.055, 0.105))
    deck_geom.merge(BoxGeometry((0.085, 0.014, 0.100)).translate(0.150, -0.055, 0.105))
    deck_geom.merge(BoxGeometry((0.080, 0.078, 0.026)).translate(0.145, 0.0, 0.069))
    deck_geom.merge(BoxGeometry((0.18, 0.012, 0.080)).translate(-0.285, 0.026, 0.085))
    deck_geom.merge(BoxGeometry((0.18, 0.012, 0.080)).translate(-0.285, -0.026, 0.085))
    deck_geom.merge(BoxGeometry((0.060, 0.064, 0.014)).translate(-0.180, 0.0, 0.057))
    deck_geom.merge(BoxGeometry((0.040, 0.014, 0.074)).translate(-0.375, 0.020, 0.125))
    deck_geom.merge(BoxGeometry((0.040, 0.014, 0.074)).translate(-0.375, -0.020, 0.125))
    deck_geom.merge(CylinderGeometry(0.006, 0.014, radial_segments=18).rotate_x(-pi / 2.0).translate(-0.375, 0.020, 0.080))
    deck_geom.merge(CylinderGeometry(0.006, 0.014, radial_segments=18).rotate_x(-pi / 2.0).translate(-0.375, -0.020, 0.080))
    deck.visual(save_mesh("scooter_deck_body", deck_geom), material=deck_body, name="deck_body")
    deck.visual(
        Box((0.31, 0.094, 0.004)),
        origin=Origin(xyz=(-0.035, 0.0, 0.067)),
        material=dark_rubber,
        name="grip_pad",
    )
    deck.visual(
        Box((0.070, 0.050, 0.008)),
        origin=Origin(xyz=(-0.375, 0.0, 0.166)),
        material=accent,
        name="rear_brake",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.62, 0.18, 0.22)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    front_fork = model.part("front_fork")
    fork_geom = BoxGeometry((0.072, 0.064, 0.038)).translate(0.014, 0.0, 0.051)
    fork_geom.merge(CylinderGeometry(0.012, 0.030, radial_segments=18).rotate_y(pi / 2.0).translate(-0.008, 0.0, 0.020))
    fork_geom.merge(BoxGeometry((0.024, 0.016, 0.050)).translate(-0.010, 0.040, 0.028))
    fork_geom.merge(BoxGeometry((0.024, 0.016, 0.050)).translate(-0.010, -0.040, 0.028))
    fork_geom.merge(
        tube_from_spline_points(
            [(0.015, 0.0, 0.036), (0.036, 0.024, 0.040), (0.060, 0.058, 0.037), (0.084, 0.090, 0.031)],
            radius=0.0105,
            samples_per_segment=16,
            radial_segments=18,
        )
    )
    fork_geom.merge(
        tube_from_spline_points(
            [(0.015, 0.0, 0.036), (0.036, -0.024, 0.040), (0.060, -0.058, 0.037), (0.084, -0.090, 0.031)],
            radius=0.0105,
            samples_per_segment=16,
            radial_segments=18,
        )
    )
    fork_geom.merge(CylinderGeometry(0.010, 0.150, radial_segments=18).rotate_x(-pi / 2.0).translate(0.052, 0.0, 0.042))
    fork_geom.merge(BoxGeometry((0.022, 0.016, 0.032)).translate(0.086, 0.090, 0.031))
    fork_geom.merge(BoxGeometry((0.022, 0.016, 0.032)).translate(0.086, -0.090, 0.031))
    fork_geom.merge(CylinderGeometry(0.007, 0.02272, radial_segments=18).rotate_x(-pi / 2.0).translate(0.086, 0.090, 0.031))
    fork_geom.merge(CylinderGeometry(0.007, 0.02272, radial_segments=18).rotate_x(-pi / 2.0).translate(0.086, -0.090, 0.031))
    sleeve_geom = BoxGeometry((0.016, 0.052, sleeve_length)).translate(0.026, 0.0, sleeve_length * 0.5)
    sleeve_geom.merge(BoxGeometry((0.010, 0.010, sleeve_length - 0.016)).translate(0.021, 0.021, 0.162))
    sleeve_geom.merge(BoxGeometry((0.010, 0.010, sleeve_length - 0.016)).translate(0.021, -0.021, 0.162))
    sleeve_geom.merge(BoxGeometry((0.020, 0.060, 0.050)).translate(0.025, 0.0, 0.028))
    sleeve_geom.merge(BoxGeometry((0.018, 0.050, 0.010)).translate(0.027, 0.0, sleeve_length + 0.005))
    sleeve_geom.rotate_y(-stem_rake).translate(*sleeve_start)
    front_fork.visual(save_mesh("scooter_fork_body", fork_geom), material=alloy, name="fork_body")
    front_fork.visual(save_mesh("scooter_outer_sleeve", sleeve_geom), material=alloy, name="outer_sleeve")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.24, 0.25, 0.40)),
        mass=1.2,
        origin=Origin(xyz=(-0.01, 0.0, 0.16)),
    )

    stem_upper = model.part("stem_upper")
    stem_upper.visual(
        Cylinder(radius=0.014, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=alloy,
        name="inner_tube",
    )
    stem_upper.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=alloy,
        name="stop_collar",
    )
    stem_upper.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=alloy,
        name="bar_clamp",
    )
    stem_upper.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.304), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="handlebar",
    )
    stem_upper.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.0, -0.135, 0.304), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="right_grip",
    )
    stem_upper.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.0, 0.135, 0.304), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="left_grip",
    )
    stem_upper.visual(
        Box((0.050, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=accent,
        name="center_pad",
    )
    stem_upper.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.48)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    left_front_wheel = add_wheel(
        "left_front_wheel",
        radius=0.12,
        width=0.028,
        mass=0.42,
        mesh_prefix="left_front_wheel",
    )
    right_front_wheel = add_wheel(
        "right_front_wheel",
        radius=0.12,
        width=0.028,
        mass=0.42,
        mesh_prefix="right_front_wheel",
    )
    rear_wheel = add_wheel(
        "rear_wheel",
        radius=0.08,
        width=0.026,
        mass=0.28,
        mesh_prefix="rear_wheel",
    )

    lean_joint = model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.180, 0.0, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=2.5,
            lower=-0.36,
            upper=0.36,
        ),
    )
    stem_slide = model.articulation(
        "fork_to_stem_upper",
        ArticulationType.PRISMATIC,
        parent=front_fork,
        child=stem_upper,
        origin=Origin(xyz=sleeve_top, rpy=(0.0, -stem_rake, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.25,
            lower=0.0,
            upper=0.10,
        ),
    )
    left_spin = model.articulation(
        "fork_to_left_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=left_front_wheel,
        origin=Origin(xyz=(0.086, 0.112, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )
    right_spin = model.articulation(
        "fork_to_right_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=right_front_wheel,
        origin=Origin(xyz=(0.086, -0.112, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )
    rear_spin = model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.375, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    deck = object_model.get_part("deck_chassis")
    front_fork = object_model.get_part("front_fork")
    stem_upper = object_model.get_part("stem_upper")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    lean_joint = object_model.get_articulation("deck_to_front_fork")
    stem_slide = object_model.get_articulation("fork_to_stem_upper")
    left_spin = object_model.get_articulation("fork_to_left_front_wheel")
    right_spin = object_model.get_articulation("fork_to_right_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")

    ctx.check(
        "all primary scooter parts exist",
        all(part is not None for part in (deck, front_fork, stem_upper, left_front_wheel, right_front_wheel, rear_wheel)),
        details="Expected deck, fork, telescoping stem, and three wheels.",
    )
    ctx.check(
        "lean and wheel joints use the intended articulation types",
        lean_joint.joint_type == ArticulationType.REVOLUTE
        and stem_slide.joint_type == ArticulationType.PRISMATIC
        and left_spin.joint_type == ArticulationType.CONTINUOUS
        and right_spin.joint_type == ArticulationType.CONTINUOUS
        and rear_spin.joint_type == ArticulationType.CONTINUOUS,
        details=(
            f"lean={lean_joint.joint_type}, stem={stem_slide.joint_type}, "
            f"left={left_spin.joint_type}, right={right_spin.joint_type}, rear={rear_spin.joint_type}"
        ),
    )
    ctx.check(
        "continuous wheel joints are unbounded",
        all(j.motion_limits is not None and j.motion_limits.lower is None and j.motion_limits.upper is None for j in (left_spin, right_spin, rear_spin)),
        details=(
            f"left={left_spin.motion_limits}, right={right_spin.motion_limits}, "
            f"rear={rear_spin.motion_limits}"
        ),
    )

    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="z",
        positive_elem="rear_brake",
        negative_elem="tire",
        min_gap=0.001,
        max_gap=0.010,
        name="rear brake sits just above the rear wheel",
    )
    ctx.expect_contact(
        deck,
        rear_wheel,
        elem_a="deck_body",
        elem_b="left_bearing_shoulder",
        contact_tol=0.0005,
        name="rear wheel is carried by the rear fork axle mount",
    )
    ctx.expect_contact(
        front_fork,
        stem_upper,
        elem_a="outer_sleeve",
        elem_b="stop_collar",
        contact_tol=0.0005,
        name="stem stop collar seats on the fork sleeve head",
    )
    ctx.expect_contact(
        front_fork,
        left_front_wheel,
        elem_a="fork_body",
        elem_b="left_bearing_shoulder",
        contact_tol=0.0005,
        name="left front wheel sits on its fork axle mount",
    )
    ctx.expect_contact(
        front_fork,
        right_front_wheel,
        elem_a="fork_body",
        elem_b="right_bearing_shoulder",
        contact_tol=0.0005,
        name="right front wheel sits on its fork axle mount",
    )

    ctx.expect_within(
        stem_upper,
        front_fork,
        axes="y",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.003,
        name="stem tube stays centered laterally in the outer sleeve",
    )

    rest_stem_pos = ctx.part_world_position(stem_upper)
    with ctx.pose({stem_slide: stem_slide.motion_limits.upper}):
        ctx.expect_within(
            stem_upper,
            front_fork,
            axes="y",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="extended stem remains centered in the sleeve",
        )
        ctx.expect_overlap(
            stem_upper,
            front_fork,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.050,
            name="extended stem retains insertion in the sleeve",
        )
        extended_stem_pos = ctx.part_world_position(stem_upper)

    ctx.check(
        "telescoping stem raises the handlebar",
        rest_stem_pos is not None
        and extended_stem_pos is not None
        and extended_stem_pos[2] > rest_stem_pos[2] + 0.085
        and extended_stem_pos[0] < rest_stem_pos[0] - 0.020,
        details=f"rest={rest_stem_pos}, extended={extended_stem_pos}",
    )

    with ctx.pose({lean_joint: 0.24}):
        left_lean_pos = ctx.part_world_position(left_front_wheel)
        right_lean_pos = ctx.part_world_position(right_front_wheel)
    ctx.check(
        "positive lean tilts the fork toward the right wheel contact side",
        left_lean_pos is not None and right_lean_pos is not None and left_lean_pos[2] > right_lean_pos[2] + 0.030,
        details=f"left={left_lean_pos}, right={right_lean_pos}",
    )

    with ctx.pose({lean_joint: -0.24}):
        left_lean_neg = ctx.part_world_position(left_front_wheel)
        right_lean_neg = ctx.part_world_position(right_front_wheel)
    ctx.check(
        "negative lean tilts the fork toward the left wheel contact side",
        left_lean_neg is not None and right_lean_neg is not None and right_lean_neg[2] > left_lean_neg[2] + 0.030,
        details=f"left={left_lean_neg}, right={right_lean_neg}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
