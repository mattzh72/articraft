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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_face_ring(
    part,
    *,
    x_center: float,
    thickness: float,
    width: float,
    height: float,
    hole_left: float,
    hole_right: float,
    hole_bottom: float,
    hole_top: float,
    material,
    prefix: str,
) -> None:
    half_width = width * 0.5
    top_height = height - hole_top
    bottom_height = hole_bottom
    middle_height = hole_top - hole_bottom
    left_width = hole_left + half_width
    right_width = half_width - hole_right

    if top_height > 0.0:
        part.visual(
            Box((thickness, width, top_height)),
            origin=Origin(xyz=(x_center, 0.0, hole_top + top_height * 0.5)),
            material=material,
            name=f"{prefix}_top_strip",
        )
    if bottom_height > 0.0:
        part.visual(
            Box((thickness, width, bottom_height)),
            origin=Origin(xyz=(x_center, 0.0, bottom_height * 0.5)),
            material=material,
            name=f"{prefix}_bottom_strip",
        )
    if left_width > 0.0 and middle_height > 0.0:
        part.visual(
            Box((thickness, left_width, middle_height)),
            origin=Origin(
                xyz=(x_center, (-half_width + hole_left) * 0.5, hole_bottom + middle_height * 0.5)
            ),
            material=material,
            name=f"{prefix}_left_strip",
        )
    if right_width > 0.0 and middle_height > 0.0:
        part.visual(
            Box((thickness, right_width, middle_height)),
            origin=Origin(
                xyz=(x_center, (hole_right + half_width) * 0.5, hole_bottom + middle_height * 0.5)
            ),
            material=material,
            name=f"{prefix}_right_strip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_pickup_tailgate")

    body_paint = model.material("body_paint", rgba=(0.71, 0.73, 0.76, 1.0))
    inner_panel = model.material("inner_panel", rgba=(0.55, 0.57, 0.60, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.30, 0.31, 0.34, 1.0))
    reflector_red = model.material("reflector_red", rgba=(0.72, 0.10, 0.10, 1.0))

    opening_width = 1.56
    bedside_thickness = 0.075
    bed_depth = 0.28
    sill_height = 0.06
    opening_height = 0.56
    top_rail_height = 0.05
    total_width = opening_width + 2.0 * bedside_thickness

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((bed_depth, total_width, sill_height)),
        origin=Origin(xyz=(bed_depth * 0.5, 0.0, sill_height * 0.5)),
        material=inner_panel,
        name="rear_sill",
    )
    for side, y_sign in (("left", -1.0), ("right", 1.0)):
        bed_frame.visual(
            Box((bed_depth, bedside_thickness, opening_height)),
            origin=Origin(
                xyz=(
                    bed_depth * 0.5,
                    y_sign * (opening_width * 0.5 + bedside_thickness * 0.5),
                    sill_height + opening_height * 0.5,
                )
            ),
            material=body_paint,
            name=f"{side}_bedside",
        )
        bed_frame.visual(
            Box((bed_depth, bedside_thickness, top_rail_height)),
            origin=Origin(
                xyz=(
                    bed_depth * 0.5,
                    y_sign * (opening_width * 0.5 + bedside_thickness * 0.5),
                    sill_height + opening_height + top_rail_height * 0.5,
                )
            ),
            material=body_paint,
            name=f"{side}_top_cap",
        )
        bed_frame.visual(
            Box((0.008, 0.028, 0.20)),
            origin=Origin(
                xyz=(
                    0.004,
                    y_sign * (opening_width * 0.5 + bedside_thickness * 0.5),
                    sill_height + 0.26,
                )
            ),
            material=reflector_red,
            name=f"{side}_taillamp",
        )
    bed_frame.visual(
        Box((0.11, total_width, 0.11)),
        origin=Origin(xyz=(0.055, 0.0, -0.055)),
        material=trim_black,
        name="step_bumper",
    )
    for index, y_center in enumerate((-0.43, 0.0, 0.43)):
        bed_frame.visual(
            Cylinder(radius=0.016, length=0.10),
            origin=Origin(
                xyz=(0.012, y_center, sill_height - 0.002),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"frame_hinge_knuckle_{index}",
        )
    bed_frame.inertial = Inertial.from_geometry(
        Box((bed_depth, total_width, sill_height + opening_height + top_rail_height)),
        mass=28.0,
        origin=Origin(
            xyz=(
                bed_depth * 0.5,
                0.0,
                (sill_height + opening_height + top_rail_height) * 0.5,
            )
        ),
    )

    gate_width = 1.52
    gate_height = 0.55
    gate_depth = 0.065
    outer_skin_t = 0.0035
    inner_skin_t = 0.003
    shell_depth = gate_depth - outer_skin_t - inner_skin_t
    shell_center_x = ((-gate_depth + outer_skin_t) + (-inner_skin_t)) * 0.5
    edge_wall_t = 0.045
    hatch_frame_t = 0.018

    hatch_open_width = 0.42
    hatch_open_height = 0.34
    hatch_open_right = gate_width * 0.5 - 0.10
    hatch_open_left = hatch_open_right - hatch_open_width
    hatch_open_bottom = 0.10
    hatch_open_top = hatch_open_bottom + hatch_open_height

    tailgate = model.part("tailgate")
    _add_face_ring(
        tailgate,
        x_center=-gate_depth + outer_skin_t * 0.5,
        thickness=outer_skin_t,
        width=gate_width,
        height=gate_height,
        hole_left=hatch_open_left,
        hole_right=hatch_open_right,
        hole_bottom=hatch_open_bottom,
        hole_top=hatch_open_top,
        material=body_paint,
        prefix="outer_face",
    )
    _add_face_ring(
        tailgate,
        x_center=-inner_skin_t * 0.5,
        thickness=inner_skin_t,
        width=gate_width,
        height=gate_height,
        hole_left=hatch_open_left,
        hole_right=hatch_open_right,
        hole_bottom=hatch_open_bottom,
        hole_top=hatch_open_top,
        material=inner_panel,
        prefix="inner_face",
    )
    tailgate.visual(
        Box((shell_depth, gate_width, 0.078)),
        origin=Origin(xyz=(shell_center_x, 0.0, 0.039)),
        material=inner_panel,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((shell_depth, gate_width, 0.070)),
        origin=Origin(xyz=(shell_center_x, 0.0, gate_height - 0.035)),
        material=inner_panel,
        name="top_rail",
    )
    tailgate.visual(
        Box((shell_depth, edge_wall_t, gate_height)),
        origin=Origin(
            xyz=(shell_center_x, -gate_width * 0.5 + edge_wall_t * 0.5, gate_height * 0.5)
        ),
        material=inner_panel,
        name="left_outer_edge",
    )
    tailgate.visual(
        Box((shell_depth, edge_wall_t, gate_height)),
        origin=Origin(
            xyz=(shell_center_x, gate_width * 0.5 - edge_wall_t * 0.5, gate_height * 0.5)
        ),
        material=inner_panel,
        name="right_outer_edge",
    )
    tailgate.visual(
        Box((shell_depth, hatch_frame_t, hatch_open_height)),
        origin=Origin(
            xyz=(
                shell_center_x,
                hatch_open_left - hatch_frame_t * 0.5,
                hatch_open_bottom + hatch_open_height * 0.5,
            )
        ),
        material=inner_panel,
        name="hatch_left_jamb",
    )
    tailgate.visual(
        Box((shell_depth, hatch_frame_t, hatch_open_height)),
        origin=Origin(
            xyz=(
                shell_center_x,
                hatch_open_right + hatch_frame_t * 0.5,
                hatch_open_bottom + hatch_open_height * 0.5,
            )
        ),
        material=inner_panel,
        name="hatch_right_jamb",
    )
    tailgate.visual(
        Box((shell_depth, hatch_open_width + 2.0 * hatch_frame_t, hatch_frame_t)),
        origin=Origin(
            xyz=(
                shell_center_x,
                (hatch_open_left + hatch_open_right) * 0.5,
                hatch_open_bottom - hatch_frame_t * 0.5,
            )
        ),
        material=inner_panel,
        name="hatch_bottom_jamb",
    )
    tailgate.visual(
        Box((shell_depth, hatch_open_width + 2.0 * hatch_frame_t, hatch_frame_t)),
        origin=Origin(
            xyz=(
                shell_center_x,
                (hatch_open_left + hatch_open_right) * 0.5,
                hatch_open_top + hatch_frame_t * 0.5,
            )
        ),
        material=inner_panel,
        name="hatch_top_jamb",
    )
    tailgate.visual(
        Box((0.028, 0.22, 0.028)),
        origin=Origin(xyz=(-gate_depth + 0.0145, -0.08, gate_height - 0.08)),
        material=trim_black,
        name="tailgate_handle_bezel",
    )
    tailgate.visual(
        Box((0.014, 0.11, 0.010)),
        origin=Origin(xyz=(-gate_depth + 0.0075, -0.08, gate_height - 0.08)),
        material=hinge_metal,
        name="tailgate_handle_pull",
    )
    for index, y_center in enumerate((-0.23, 0.23)):
        tailgate.visual(
            Cylinder(radius=0.015, length=0.17),
            origin=Origin(
                xyz=(-0.018, y_center, 0.000),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"tailgate_hinge_knuckle_{index}",
        )
    tailgate.inertial = Inertial.from_geometry(
        Box((gate_depth, gate_width, gate_height)),
        mass=18.0,
        origin=Origin(xyz=(-gate_depth * 0.5, 0.0, gate_height * 0.5)),
    )

    hatch_gap = 0.0035
    hatch_depth = 0.053
    hatch_shell_t = 0.0035
    hatch_inner_t = 0.003
    hatch_leaf_width = hatch_open_width - 2.0 * hatch_gap
    hatch_leaf_height = hatch_open_height - 2.0 * hatch_gap
    hatch_inner_depth = hatch_depth - hatch_shell_t - hatch_inner_t
    hatch_inner_center_x = hatch_inner_depth * 0.5

    utility_hatch = model.part("utility_hatch")
    utility_hatch.visual(
        Box((hatch_shell_t, hatch_leaf_width, hatch_leaf_height)),
        origin=Origin(
            xyz=(-hatch_shell_t * 0.5, -hatch_leaf_width * 0.5, 0.0),
        ),
        material=body_paint,
        name="door_outer_skin",
    )
    utility_hatch.visual(
        Box((hatch_inner_t, hatch_leaf_width, hatch_leaf_height)),
        origin=Origin(
            xyz=(hatch_inner_depth - hatch_inner_t * 0.5, -hatch_leaf_width * 0.5, 0.0),
        ),
        material=inner_panel,
        name="door_inner_skin",
    )
    utility_hatch.visual(
        Box((hatch_inner_depth, 0.020, hatch_leaf_height)),
        origin=Origin(xyz=(hatch_inner_center_x, -0.010, 0.0)),
        material=inner_panel,
        name="door_hinge_stile",
    )
    utility_hatch.visual(
        Box((hatch_inner_depth, 0.020, hatch_leaf_height)),
        origin=Origin(
            xyz=(hatch_inner_center_x, -hatch_leaf_width + 0.010, 0.0),
        ),
        material=inner_panel,
        name="door_free_stile",
    )
    utility_hatch.visual(
        Box((hatch_inner_depth, hatch_leaf_width, 0.020)),
        origin=Origin(
            xyz=(hatch_inner_center_x, -hatch_leaf_width * 0.5, hatch_leaf_height * 0.5 - 0.010),
        ),
        material=inner_panel,
        name="door_top_rail",
    )
    utility_hatch.visual(
        Box((hatch_inner_depth, hatch_leaf_width, 0.020)),
        origin=Origin(
            xyz=(hatch_inner_center_x, -hatch_leaf_width * 0.5, -hatch_leaf_height * 0.5 + 0.010),
        ),
        material=inner_panel,
        name="door_bottom_rail",
    )
    utility_hatch.visual(
        Box((0.010, 0.075, 0.020)),
        origin=Origin(
            xyz=(-0.003, -hatch_leaf_width + 0.065, 0.0),
        ),
        material=trim_black,
        name="door_pull",
    )
    utility_hatch.visual(
        Cylinder(radius=0.009, length=0.034),
        origin=Origin(
            xyz=(-0.008, -0.004, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_metal,
        name="door_hinge_barrel",
    )
    utility_hatch.inertial = Inertial.from_geometry(
        Box((hatch_depth, hatch_leaf_width, hatch_leaf_height)),
        mass=3.2,
        origin=Origin(xyz=(hatch_depth * 0.5, -hatch_leaf_width * 0.5, 0.0)),
    )

    tailgate_hinge = model.articulation(
        "bed_frame_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, sill_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    utility_hinge = model.articulation(
        "tailgate_to_utility_hatch",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=utility_hatch,
        origin=Origin(
            xyz=(
                -gate_depth + outer_skin_t,
                hatch_open_right - hatch_gap,
                hatch_open_bottom + hatch_open_height * 0.5,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    model.meta["prompt_articulations"] = {
        "tailgate_hinge": tailgate_hinge.name,
        "utility_hinge": utility_hinge.name,
    }
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

    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    utility_hatch = object_model.get_part("utility_hatch")
    tailgate_hinge = object_model.get_articulation("bed_frame_to_tailgate")
    utility_hinge = object_model.get_articulation("tailgate_to_utility_hatch")

    ctx.check(
        "tailgate hinge axis is horizontal",
        tuple(tailgate_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tailgate_hinge.axis}",
    )
    ctx.check(
        "utility hatch hinge axis is vertical",
        tuple(utility_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={utility_hinge.axis}",
    )

    with ctx.pose({tailgate_hinge: 0.0, utility_hinge: 0.0}):
        ctx.expect_overlap(
            tailgate,
            bed_frame,
            axes="yz",
            min_overlap=0.50,
            name="closed tailgate covers the rear opening",
        )
        ctx.expect_within(
            utility_hatch,
            tailgate,
            axes="yz",
            margin=0.01,
            name="closed utility hatch stays within the tailgate envelope",
        )
        ctx.expect_gap(
            tailgate,
            utility_hatch,
            axis="y",
            positive_elem="hatch_right_jamb",
            negative_elem="door_outer_skin",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed utility hatch keeps a small right-side reveal",
        )

        closed_tailgate_aabb = ctx.part_world_aabb(tailgate)
        closed_hatch_aabb = ctx.part_world_aabb(utility_hatch)

    with ctx.pose({tailgate_hinge: math.radians(92.0), utility_hinge: 0.0}):
        open_tailgate_aabb = ctx.part_world_aabb(tailgate)

    with ctx.pose({tailgate_hinge: 0.0, utility_hinge: math.radians(85.0)}):
        open_hatch_aabb = ctx.part_world_aabb(utility_hatch)

    ctx.check(
        "tailgate opens downward behind the bed",
        closed_tailgate_aabb is not None
        and open_tailgate_aabb is not None
        and open_tailgate_aabb[0][0] < closed_tailgate_aabb[0][0] - 0.40
        and open_tailgate_aabb[1][2] < closed_tailgate_aabb[1][2] - 0.20,
        details=f"closed={closed_tailgate_aabb}, open={open_tailgate_aabb}",
    )
    ctx.check(
        "utility hatch swings outward from the tailgate face",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][0] < closed_hatch_aabb[0][0] - 0.10,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
