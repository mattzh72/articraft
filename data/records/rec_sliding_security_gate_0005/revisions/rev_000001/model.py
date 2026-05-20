from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_security_gate", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.58, 0.58, 0.60, 1.0))
    powder_charcoal = model.material("powder_charcoal", rgba=(0.16, 0.18, 0.20, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    satin_stainless = model.material("satin_stainless", rgba=(0.69, 0.71, 0.73, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.10, 0.11, 1.0))

    def rounded_member_mesh(
        filename: str,
        size: tuple[float, float, float],
        *,
        axis: str = "z",
        radius: float | None = None,
    ):
        sx, sy, sz = size
        edge_radius = radius if radius is not None else min(sx, sy) * 0.22
        if axis == "z":
            geom = ExtrudeGeometry(rounded_rect_profile(sx, sy, edge_radius), sz, center=True)
        elif axis == "x":
            geom = ExtrudeGeometry(rounded_rect_profile(sz, sy, edge_radius), sx, center=True)
            geom.rotate_y(math.pi / 2.0)
        elif axis == "y":
            geom = ExtrudeGeometry(rounded_rect_profile(sx, sz, edge_radius), sy, center=True)
            geom.rotate_x(-math.pi / 2.0)
        else:
            raise ValueError(f"Unsupported axis {axis!r}")
        return mesh_from_geometry(geom, ASSETS.mesh_path(filename))

    rail_top_z = 0.118
    wheel_radius = 0.065
    wheel_center_z = rail_top_z + wheel_radius

    track_base = model.part("track_base")
    track_base.visual(
        Box((6.60, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="plinth",
    )
    track_base.visual(
        Box((6.40, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=satin_graphite,
        name="rail_bed",
    )
    track_base.visual(
        Box((6.20, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=satin_stainless,
        name="rail_cap",
    )
    track_base.inertial = Inertial.from_geometry(
        Box((6.60, 0.34, 0.118)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    left_post = model.part("left_post")
    left_post.visual(
        rounded_member_mesh("left_post_body.obj", (0.18, 0.12, 2.51), axis="z", radius=0.016),
        origin=Origin(xyz=(-0.18, -0.15, 1.375)),
        material=powder_charcoal,
        name="left_post_body",
    )
    left_post.visual(
        Box((0.30, 0.10, 0.04)),
        origin=Origin(xyz=(-0.18, -0.15, 0.10)),
        material=satin_graphite,
        name="left_base_shoe",
    )
    left_post.visual(
        rounded_member_mesh("left_guide_bridge.obj", (0.46, 0.12, 0.06), axis="x", radius=0.014),
        origin=Origin(xyz=(0.10, -0.02, 2.26)),
        material=powder_charcoal,
        name="guide_bridge",
    )
    left_post.visual(
        Box((0.16, 0.10, 0.44)),
        origin=Origin(xyz=(-0.02, -0.09, 2.08)),
        material=powder_charcoal,
        name="bridge_support",
    )
    for index, x_pos in enumerate((0.08, 0.20), start=1):
        left_post.visual(
            Box((0.07, 0.008, 0.10)),
            origin=Origin(xyz=(x_pos, 0.024, 2.22)),
            material=satin_graphite,
            name=f"guide_clevis_{index}_pos",
        )
        left_post.visual(
            Box((0.07, 0.008, 0.10)),
            origin=Origin(xyz=(x_pos, -0.024, 2.22)),
            material=satin_graphite,
            name=f"guide_clevis_{index}_neg",
        )
    left_post.inertial = Inertial.from_geometry(
        Box((0.64, 0.24, 2.55)),
        mass=58.0,
        origin=Origin(xyz=(0.02, -0.08, 1.275)),
    )

    right_post = model.part("right_post")
    right_post.visual(
        rounded_member_mesh("right_post_body.obj", (0.16, 0.12, 2.51), axis="z", radius=0.016),
        origin=Origin(xyz=(3.10, 0.15, 1.375)),
        material=powder_charcoal,
        name="right_post_body",
    )
    right_post.visual(
        Box((0.28, 0.10, 0.04)),
        origin=Origin(xyz=(3.10, 0.15, 0.10)),
        material=satin_graphite,
        name="right_base_shoe",
    )
    right_post.visual(
        Box((0.02, 0.22, 0.28)),
        origin=Origin(xyz=(3.03, 0.06, 1.06)),
        material=satin_stainless,
        name="keeper_back",
    )
    right_post.visual(
        Box((0.06, 0.008, 0.28)),
        origin=Origin(xyz=(3.05, 0.044, 1.06)),
        material=satin_stainless,
        name="keeper_side_pos",
    )
    right_post.visual(
        Box((0.06, 0.008, 0.28)),
        origin=Origin(xyz=(3.05, -0.044, 1.06)),
        material=satin_stainless,
        name="keeper_side_neg",
    )
    right_post.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 2.55)),
        mass=52.0,
        origin=Origin(xyz=(3.10, 0.10, 1.275)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        rounded_member_mesh("gate_trailing_stile.obj", (0.12, 0.06, 1.62), axis="z", radius=0.011),
        origin=Origin(xyz=(0.06, 0.0, 1.23)),
        material=powder_charcoal,
        name="trailing_stile",
    )
    gate_leaf.visual(
        rounded_member_mesh("gate_leading_stile.obj", (0.12, 0.06, 1.62), axis="z", radius=0.011),
        origin=Origin(xyz=(2.94, 0.0, 1.23)),
        material=powder_charcoal,
        name="leading_stile",
    )
    gate_leaf.visual(
        rounded_member_mesh("gate_top_rail.obj", (3.00, 0.06, 0.10), axis="x", radius=0.011),
        origin=Origin(xyz=(1.50, 0.0, 2.09)),
        material=powder_charcoal,
        name="top_rail",
    )
    gate_leaf.visual(
        rounded_member_mesh("gate_bottom_rail.obj", (3.00, 0.06, 0.16), axis="x", radius=0.011),
        origin=Origin(xyz=(1.50, 0.0, 0.33)),
        material=powder_charcoal,
        name="bottom_rail",
    )
    gate_leaf.visual(
        rounded_member_mesh("gate_mid_rail.obj", (2.76, 0.06, 0.08), axis="x", radius=0.010),
        origin=Origin(xyz=(1.50, 0.0, 1.02)),
        material=powder_charcoal,
        name="mid_rail",
    )
    gate_leaf.visual(
        Box((2.72, 0.018, 0.57)),
        origin=Origin(xyz=(1.50, 0.0, 0.695)),
        material=satin_graphite,
        name="lower_panel",
    )
    slat_x_positions = (0.34, 0.63, 0.92, 1.21, 1.50, 1.79, 2.08, 2.37, 2.66)
    for index, x_pos in enumerate(slat_x_positions, start=1):
        slat_width = 0.09 if index == 5 else 0.06
        slat_material = satin_stainless if index == 5 else satin_graphite
        gate_leaf.visual(
            Box((slat_width, 0.022, 1.02)),
            origin=Origin(xyz=(x_pos, 0.0, 1.55)),
            material=slat_material,
            name=f"upper_slat_{index}",
        )
    gate_leaf.visual(
        Box((0.16, 0.05, 0.22)),
        origin=Origin(xyz=(2.89, 0.0, 1.06)),
        material=satin_graphite,
        name="lock_housing",
    )
    gate_leaf.visual(
        Box((0.006, 0.052, 0.22)),
        origin=Origin(xyz=(2.999, 0.0, 1.06)),
        material=satin_stainless,
        name="latch_faceplate",
    )
    gate_leaf.visual(
        Box((0.018, 0.008, 1.72)),
        origin=Origin(xyz=(2.99, 0.026, 1.23)),
        material=satin_stainless,
        name="edge_trim_pos",
    )
    gate_leaf.visual(
        Box((0.018, 0.008, 1.72)),
        origin=Origin(xyz=(2.99, -0.026, 1.23)),
        material=satin_stainless,
        name="edge_trim_neg",
    )
    gate_leaf.visual(
        Cylinder(radius=0.010, length=0.34),
        origin=Origin(xyz=(2.72, 0.040, 1.06)),
        material=satin_stainless,
        name="pull_handle_grip",
    )
    gate_leaf.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(2.72, 0.032, 1.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="pull_handle_standoff_top",
    )
    gate_leaf.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(2.72, 0.032, 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="pull_handle_standoff_bottom",
    )
    for index, x_pos in enumerate((0.72, 2.28), start=1):
        gate_leaf.visual(
            Box((0.10, 0.008, 0.16)),
            origin=Origin(xyz=(x_pos, 0.024, 0.20)),
            material=powder_charcoal,
            name=f"wheel_carriage_{index}_pos",
        )
        gate_leaf.visual(
            Box((0.10, 0.008, 0.16)),
            origin=Origin(xyz=(x_pos, -0.024, 0.20)),
            material=powder_charcoal,
            name=f"wheel_carriage_{index}_neg",
        )
        gate_leaf.visual(
            Box((0.12, 0.04, 0.08)),
            origin=Origin(xyz=(x_pos, 0.0, 0.30)),
            material=powder_charcoal,
            name=f"wheel_carriage_bridge_{index}",
        )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((3.00, 0.06, 2.14)),
        mass=112.0,
        origin=Origin(xyz=(1.50, 0.0, 1.07)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=wheel_radius, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="front_wheel_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="front_wheel_cap_pos",
    )
    front_wheel.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="front_wheel_cap_neg",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.04),
        mass=3.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=wheel_radius, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="rear_wheel_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="rear_wheel_cap_pos",
    )
    rear_wheel.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="rear_wheel_cap_neg",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.04),
        mass=3.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    top_guide_front = model.part("top_guide_front")
    top_guide_front.visual(
        Cylinder(radius=0.03, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="top_guide_front_roller",
    )
    top_guide_front.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="top_guide_front_cap_pos",
    )
    top_guide_front.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="top_guide_front_cap_neg",
    )
    top_guide_front.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.04),
        mass=0.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    top_guide_rear = model.part("top_guide_rear")
    top_guide_rear.visual(
        Cylinder(radius=0.03, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="top_guide_rear_roller",
    )
    top_guide_rear.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="top_guide_rear_cap_pos",
    )
    top_guide_rear.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_stainless,
        name="top_guide_rear_cap_neg",
    )
    top_guide_rear.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.04),
        mass=0.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "track_to_left_post",
        ArticulationType.FIXED,
        parent=track_base,
        child=left_post,
    )
    model.articulation(
        "track_to_right_post",
        ArticulationType.FIXED,
        parent=track_base,
        child=right_post,
    )
    model.articulation(
        "track_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=track_base,
        child=gate_leaf,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.45,
            lower=-2.75,
            upper=0.0,
        ),
    )
    model.articulation(
        "gate_leaf_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=gate_leaf,
        child=front_wheel,
        origin=Origin(xyz=(0.72, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "gate_leaf_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=gate_leaf,
        child=rear_wheel,
        origin=Origin(xyz=(2.28, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "left_post_to_top_guide_front",
        ArticulationType.CONTINUOUS,
        parent=left_post,
        child=top_guide_front,
        origin=Origin(xyz=(0.08, 0.0, 2.17)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=16.0),
    )
    model.articulation(
        "left_post_to_top_guide_rear",
        ArticulationType.CONTINUOUS,
        parent=left_post,
        child=top_guide_rear,
        origin=Origin(xyz=(0.20, 0.0, 2.17)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    track_base = object_model.get_part("track_base")
    left_post = object_model.get_part("left_post")
    right_post = object_model.get_part("right_post")
    gate_leaf = object_model.get_part("gate_leaf")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    top_guide_front = object_model.get_part("top_guide_front")
    top_guide_rear = object_model.get_part("top_guide_rear")

    gate_slide = object_model.get_articulation("track_to_gate_leaf")
    front_wheel_spin = object_model.get_articulation("gate_leaf_to_front_wheel")
    rear_wheel_spin = object_model.get_articulation("gate_leaf_to_rear_wheel")
    top_guide_front_spin = object_model.get_articulation("left_post_to_top_guide_front")
    top_guide_rear_spin = object_model.get_articulation("left_post_to_top_guide_rear")

    plinth = track_base.get_visual("plinth")
    rail_cap = track_base.get_visual("rail_cap")
    left_base_shoe = left_post.get_visual("left_base_shoe")
    right_base_shoe = right_post.get_visual("right_base_shoe")
    keeper_back = right_post.get_visual("keeper_back")
    latch_faceplate = gate_leaf.get_visual("latch_faceplate")
    top_rail = gate_leaf.get_visual("top_rail")
    front_wheel_tire = front_wheel.get_visual("front_wheel_tire")
    rear_wheel_tire = rear_wheel.get_visual("rear_wheel_tire")
    top_guide_front_roller = top_guide_front.get_visual("top_guide_front_roller")
    top_guide_rear_roller = top_guide_rear.get_visual("top_guide_rear_roller")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "gate_slide_axis_is_x",
        gate_slide.axis == (1.0, 0.0, 0.0),
        details=f"axis={gate_slide.axis}",
    )
    ctx.check(
        "front_wheel_axis_is_y",
        front_wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={front_wheel_spin.axis}",
    )
    ctx.check(
        "rear_wheel_axis_is_y",
        rear_wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={rear_wheel_spin.axis}",
    )
    ctx.check(
        "top_guide_front_axis_is_y",
        top_guide_front_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={top_guide_front_spin.axis}",
    )
    ctx.check(
        "top_guide_rear_axis_is_y",
        top_guide_rear_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={top_guide_rear_spin.axis}",
    )

    ctx.expect_contact(left_post, track_base, elem_a=left_base_shoe, elem_b=plinth)
    ctx.expect_contact(right_post, track_base, elem_a=right_base_shoe, elem_b=plinth)

    ctx.expect_contact(front_wheel, gate_leaf)
    ctx.expect_contact(rear_wheel, gate_leaf)
    ctx.expect_contact(front_wheel, track_base, elem_a=front_wheel_tire, elem_b=rail_cap)
    ctx.expect_contact(rear_wheel, track_base, elem_a=rear_wheel_tire, elem_b=rail_cap)

    ctx.expect_contact(top_guide_front, left_post)
    ctx.expect_contact(top_guide_rear, left_post)
    ctx.expect_gap(
        top_guide_front,
        gate_leaf,
        axis="z",
        positive_elem=top_guide_front_roller,
        negative_elem=top_rail,
        min_gap=0.0,
        max_gap=0.0002,
    )
    ctx.expect_gap(
        top_guide_rear,
        gate_leaf,
        axis="z",
        positive_elem=top_guide_rear_roller,
        negative_elem=top_rail,
        min_gap=0.0,
        max_gap=0.0002,
    )
    ctx.expect_overlap(
        top_guide_front,
        gate_leaf,
        axes="xy",
        elem_a=top_guide_front_roller,
        elem_b=top_rail,
        min_overlap=0.02,
    )
    ctx.expect_overlap(
        top_guide_rear,
        gate_leaf,
        axes="xy",
        elem_a=top_guide_rear_roller,
        elem_b=top_rail,
        min_overlap=0.02,
    )

    ctx.expect_gap(
        gate_leaf,
        track_base,
        axis="z",
        positive_elem=gate_leaf.get_visual("bottom_rail"),
        negative_elem=rail_cap,
        min_gap=0.12,
        max_gap=0.16,
    )

    gate_rest = ctx.part_world_position(gate_leaf)
    if gate_rest is None:
        ctx.fail("gate_leaf_position_available", "Gate leaf world position unavailable.")
    else:
        with ctx.pose({gate_slide: -2.75}):
            gate_open = ctx.part_world_position(gate_leaf)
            if gate_open is None:
                ctx.fail("gate_leaf_open_position_available", "Open-pose world position unavailable.")
            else:
                ctx.check(
                    "gate_leaf_translates_left_when_opening",
                    gate_open[0] < gate_rest[0] - 2.70,
                    details=f"rest={gate_rest}, open={gate_open}",
                )

    with ctx.pose({gate_slide: 0.0}):
        ctx.expect_gap(
            right_post,
            gate_leaf,
            axis="x",
            positive_elem=keeper_back,
            negative_elem=latch_faceplate,
            min_gap=0.012,
            max_gap=0.03,
        )
        ctx.expect_overlap(
            right_post,
            gate_leaf,
            axes="y",
            elem_a=keeper_back,
            elem_b=latch_faceplate,
            min_overlap=0.05,
        )
        ctx.expect_overlap(
            right_post,
            gate_leaf,
            axes="z",
            elem_a=keeper_back,
            elem_b=latch_faceplate,
            min_overlap=0.20,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_no_overlaps")
        ctx.fail_if_isolated_parts(name="closed_pose_no_floating")

    slide_limits = gate_slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({gate_slide: slide_limits.lower}):
            ctx.expect_contact(front_wheel, track_base, elem_a=front_wheel_tire, elem_b=rail_cap)
            ctx.expect_contact(rear_wheel, track_base, elem_a=rear_wheel_tire, elem_b=rail_cap)
            ctx.expect_gap(
                top_guide_front,
                gate_leaf,
                axis="z",
                positive_elem=top_guide_front_roller,
                negative_elem=top_rail,
                min_gap=0.0,
                max_gap=0.0002,
            )
            ctx.expect_gap(
                top_guide_rear,
                gate_leaf,
                axis="z",
                positive_elem=top_guide_rear_roller,
                negative_elem=top_rail,
                min_gap=0.0,
                max_gap=0.0002,
            )
            ctx.expect_gap(
                right_post,
                gate_leaf,
                axis="x",
                min_gap=2.5,
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_no_overlaps")
            ctx.fail_if_isolated_parts(name="open_pose_no_floating")
        with ctx.pose({gate_slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_upper_limit_no_overlaps")
            ctx.fail_if_isolated_parts(name="slide_upper_limit_no_floating")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
