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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _louver_profile(chord: float, thickness: float) -> list[tuple[float, float]]:
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    return [
        (-half_chord, 0.0),
        (-chord * 0.34, half_thickness * 0.88),
        (0.0, half_thickness),
        (chord * 0.34, half_thickness * 0.72),
        (half_chord, 0.0),
        (chord * 0.24, -half_thickness * 0.74),
        (-chord * 0.14, -half_thickness),
        (-chord * 0.42, -half_thickness * 0.46),
    ]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[idx] + maxs[idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bifold_louvered_shutter")

    painted_wood = model.material("painted_wood", rgba=(0.95, 0.95, 0.92, 1.0))
    louver_finish = model.material("louver_finish", rgba=(0.93, 0.93, 0.90, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.68, 0.70, 0.72, 1.0))

    opening_width = 0.500
    opening_height = 1.200
    frame_depth = 0.050
    frame_face = 0.030
    frame_overlap = 0.004
    hinge_barrel_radius = 0.005
    hinge_barrel_length = 0.130
    hinge_barrel_y = -0.0185

    leaf_height = 1.180
    leaf_depth = 0.028
    stile_width = 0.030
    rail_height = 0.090
    side_gap = 0.000
    fold_gap = 0.000
    leaf_width = (opening_width - (2.0 * side_gap) - fold_gap) * 0.5
    corner_overlap = 0.004

    louver_count = 11
    louver_chord = 0.075
    louver_thickness = 0.009
    louver_pin_radius = 0.0026
    louver_pin_length = 0.006
    louver_pin_overlap = 0.001
    louver_side_clearance_total = 0.000
    louver_clearance_ends = 0.010
    louver_motion = MotionLimits(
        effort=1.5,
        velocity=3.0,
        lower=-0.65,
        upper=0.65,
    )

    louver_clear_width = leaf_width - (2.0 * stile_width)
    louver_body_length = (
        louver_clear_width
        - (2.0 * (louver_pin_length - louver_pin_overlap))
        - louver_side_clearance_total
    )
    louver_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            _louver_profile(louver_chord, louver_thickness),
            louver_body_length,
            cap=True,
            closed=True,
        ),
        "shutter_louver_blade",
    )

    frame = model.part("window_frame")
    frame.visual(
        Box((frame_face, frame_depth, opening_height + (2.0 * frame_face))),
        origin=Origin(
            xyz=(
                -((opening_width * 0.5) + (frame_face * 0.5)),
                0.0,
                0.0,
            )
        ),
        material=painted_wood,
        name="frame_left_jamb",
    )
    frame.visual(
        Box((frame_face, frame_depth, opening_height + (2.0 * frame_face))),
        origin=Origin(
            xyz=(
                (opening_width * 0.5) + (frame_face * 0.5),
                0.0,
                0.0,
            )
        ),
        material=painted_wood,
        name="frame_right_jamb",
    )
    frame.visual(
        Box((opening_width + frame_overlap, frame_depth, frame_face)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (opening_height * 0.5) + (frame_face * 0.5),
            )
        ),
        material=painted_wood,
        name="frame_header",
    )
    frame.visual(
        Box((opening_width + frame_overlap, frame_depth, frame_face)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -((opening_height * 0.5) + (frame_face * 0.5)),
            )
        ),
        material=painted_wood,
        name="frame_sill",
    )
    for z_center in (-0.37, 0.0, 0.37):
        frame.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(
                    -((opening_width * 0.5) + (hinge_barrel_radius)),
                    hinge_barrel_y,
                    z_center,
                )
            ),
            material=hinge_metal,
            name=f"frame_hinge_knuckle_{int((z_center + 0.37) * 100):02d}",
        )

    def add_leaf(leaf_name: str, left_name: str, right_name: str):
        leaf = model.part(leaf_name)
        leaf.visual(
            Box((stile_width, leaf_depth, leaf_height)),
            origin=Origin(xyz=(stile_width * 0.5, 0.0, 0.0)),
            material=painted_wood,
            name=left_name,
        )
        leaf.visual(
            Box((stile_width, leaf_depth, leaf_height)),
            origin=Origin(xyz=(leaf_width - (stile_width * 0.5), 0.0, 0.0)),
            material=painted_wood,
            name=right_name,
        )
        rail_span = leaf_width - (2.0 * stile_width) + corner_overlap
        leaf.visual(
            Box((rail_span, leaf_depth, rail_height)),
            origin=Origin(
                xyz=(leaf_width * 0.5, 0.0, (leaf_height * 0.5) - (rail_height * 0.5))
            ),
            material=painted_wood,
            name=f"{leaf_name}_top_rail",
        )
        leaf.visual(
            Box((rail_span, leaf_depth, rail_height)),
            origin=Origin(
                xyz=(leaf_width * 0.5, 0.0, -((leaf_height * 0.5) - (rail_height * 0.5)))
            ),
            material=painted_wood,
            name=f"{leaf_name}_bottom_rail",
        )
        return leaf

    outer_leaf = add_leaf(
        "outer_leaf",
        left_name="outer_hinge_stile",
        right_name="outer_fold_stile",
    )
    inner_leaf = add_leaf(
        "inner_leaf",
        left_name="inner_fold_stile",
        right_name="inner_free_stile",
    )

    for z_center in (-0.185, 0.185):
        outer_leaf.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(hinge_barrel_radius, hinge_barrel_y, z_center),
            ),
            material=hinge_metal,
            name=f"outer_frame_hinge_leaf_knuckle_{int((z_center + 0.185) * 100):02d}",
        )
    for z_center in (-0.37, 0.0, 0.37):
        outer_leaf.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(leaf_width - hinge_barrel_radius, hinge_barrel_y, z_center),
            ),
            material=hinge_metal,
            name=f"outer_fold_hinge_knuckle_{int((z_center + 0.37) * 100):02d}",
        )
    for z_center in (-0.185, 0.185):
        inner_leaf.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(hinge_barrel_radius, hinge_barrel_y, z_center),
            ),
            material=hinge_metal,
            name=f"inner_fold_hinge_knuckle_{int((z_center + 0.185) * 100):02d}",
        )

    model.articulation(
        "frame_to_outer_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=outer_leaf,
        origin=Origin(xyz=(-(opening_width * 0.5) + side_gap, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "outer_leaf_to_inner_leaf",
        ArticulationType.REVOLUTE,
        parent=outer_leaf,
        child=inner_leaf,
        origin=Origin(xyz=(leaf_width + fold_gap, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=3.00,
        ),
    )

    def add_louvers(leaf, prefix: str) -> None:
        louver_zone_height = leaf_height - (2.0 * rail_height) - (2.0 * louver_clearance_ends)
        louver_pitch = (louver_zone_height - louver_chord) / (louver_count - 1)
        bottom_center = (
            -((leaf_height * 0.5) - rail_height - louver_clearance_ends - (louver_chord * 0.5))
        )

        for idx in range(louver_count):
            louver_name = f"{prefix}_louver_{idx + 1:02d}"
            louver = model.part(louver_name)
            louver.visual(
                louver_mesh,
                origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
                material=louver_finish,
                name="louver_blade",
            )

            pin_center_offset = (louver_body_length * 0.5) + (louver_pin_length * 0.5) - louver_pin_overlap
            for pin_name, pin_x in (
                ("pivot_pin_left", -pin_center_offset),
                ("pivot_pin_right", pin_center_offset),
            ):
                louver.visual(
                    Cylinder(radius=louver_pin_radius, length=louver_pin_length),
                    origin=Origin(
                        xyz=(pin_x, 0.0, 0.0),
                        rpy=(0.0, math.pi * 0.5, 0.0),
                    ),
                    material=louver_finish,
                    name=pin_name,
                )

            z_pos = bottom_center + (idx * louver_pitch)
            model.articulation(
                f"{prefix}_to_{louver_name}",
                ArticulationType.REVOLUTE,
                parent=leaf,
                child=louver,
                origin=Origin(xyz=(leaf_width * 0.5, 0.0, z_pos)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=louver_motion,
            )

    add_louvers(outer_leaf, "outer_leaf")
    add_louvers(inner_leaf, "inner_leaf")

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

    frame = object_model.get_part("window_frame")
    outer_leaf = object_model.get_part("outer_leaf")
    inner_leaf = object_model.get_part("inner_leaf")
    outer_hinge = object_model.get_articulation("frame_to_outer_leaf")
    inner_hinge = object_model.get_articulation("outer_leaf_to_inner_leaf")
    outer_mid_louver = object_model.get_part("outer_leaf_louver_06")
    outer_mid_louver_joint = object_model.get_articulation("outer_leaf_to_outer_leaf_louver_06")

    ctx.check("frame part exists", frame is not None)
    ctx.check("outer leaf exists", outer_leaf is not None)
    ctx.check("inner leaf exists", inner_leaf is not None)
    ctx.check("representative louver exists", outer_mid_louver is not None)

    outer_axis = tuple(round(v, 6) for v in outer_hinge.axis)
    inner_axis = tuple(round(v, 6) for v in inner_hinge.axis)
    louver_axis = tuple(round(v, 6) for v in outer_mid_louver_joint.axis)
    ctx.check(
        "outer hinge axis is vertical",
        outer_axis == (0.0, 0.0, 1.0),
        details=f"axis={outer_hinge.axis}",
    )
    ctx.check(
        "inner hinge axis is vertical",
        inner_axis == (0.0, 0.0, 1.0),
        details=f"axis={inner_hinge.axis}",
    )
    ctx.check(
        "louver axis follows blade length",
        louver_axis == (1.0, 0.0, 0.0),
        details=f"axis={outer_mid_louver_joint.axis}",
    )

    with ctx.pose({outer_hinge: 0.0, inner_hinge: 0.0}):
        ctx.expect_contact(
            outer_leaf,
            frame,
            elem_a="outer_hinge_stile",
            elem_b="frame_left_jamb",
            name="outer leaf meets the frame at the outer hinge line",
        )
        ctx.expect_contact(
            inner_leaf,
            outer_leaf,
            elem_a="inner_fold_stile",
            elem_b="outer_fold_stile",
            name="leaf pair meets at the fold hinge line",
        )
        ctx.expect_contact(
            frame,
            inner_leaf,
            elem_a="frame_right_jamb",
            elem_b="inner_free_stile",
            name="inner leaf meets the frame at the free-side stop line",
        )
        ctx.expect_overlap(
            outer_leaf,
            inner_leaf,
            axes="yz",
            min_overlap=0.025,
            name="closed leaves align across height and depth",
        )

    outer_fold_closed = None
    outer_fold_open = None
    with ctx.pose({outer_hinge: 0.0}):
        outer_fold_closed = _aabb_center(
            ctx.part_element_world_aabb(outer_leaf, elem="outer_fold_stile")
        )
    with ctx.pose({outer_hinge: 1.00}):
        outer_fold_open = _aabb_center(
            ctx.part_element_world_aabb(outer_leaf, elem="outer_fold_stile")
        )
    ctx.check(
        "outer leaf swings outward from frame hinge",
        outer_fold_closed is not None
        and outer_fold_open is not None
        and outer_fold_open[1] > outer_fold_closed[1] + 0.12,
        details=f"closed={outer_fold_closed}, open={outer_fold_open}",
    )

    inner_free_closed = None
    inner_free_folded = None
    with ctx.pose({outer_hinge: 0.0, inner_hinge: 0.0}):
        inner_free_closed = _aabb_center(
            ctx.part_element_world_aabb(inner_leaf, elem="inner_free_stile")
        )
    with ctx.pose({outer_hinge: 0.90, inner_hinge: 1.40}):
        inner_free_folded = _aabb_center(
            ctx.part_element_world_aabb(inner_leaf, elem="inner_free_stile")
        )
    ctx.check(
        "inner leaf folds around the center hinge",
        inner_free_closed is not None
        and inner_free_folded is not None
        and inner_free_folded[1] > inner_free_closed[1] + 0.18,
        details=f"closed={inner_free_closed}, folded={inner_free_folded}",
    )

    louver_closed = None
    louver_open = None
    with ctx.pose({outer_mid_louver_joint: 0.0}):
        louver_closed = ctx.part_element_world_aabb(outer_mid_louver, elem="louver_blade")
    with ctx.pose({outer_mid_louver_joint: 0.55}):
        louver_open = ctx.part_element_world_aabb(outer_mid_louver, elem="louver_blade")

    def size_y(aabb):
        if aabb is None:
            return None
        return aabb[1][1] - aabb[0][1]

    def size_z(aabb):
        if aabb is None:
            return None
        return aabb[1][2] - aabb[0][2]

    ctx.check(
        "representative louver changes section when rotated",
        louver_closed is not None
        and louver_open is not None
        and size_y(louver_open) is not None
        and size_y(louver_closed) is not None
        and size_z(louver_open) is not None
        and size_z(louver_closed) is not None
        and size_y(louver_open) > size_y(louver_closed) + 0.015
        and size_z(louver_open) < size_z(louver_closed) - 0.004,
        details=f"closed={louver_closed}, open={louver_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
