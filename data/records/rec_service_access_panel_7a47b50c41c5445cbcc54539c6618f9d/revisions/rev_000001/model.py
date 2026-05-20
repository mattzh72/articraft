from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    frame_outer_w = 0.68
    frame_outer_h = 0.54
    opening_w = 0.52
    opening_h = 0.38
    frame_thickness = 0.016
    cavity_depth = 0.11
    liner_wall = 0.02

    side_gap = 0.004
    top_gap = 0.004
    door_w = opening_w - 2.0 * side_gap
    door_h = opening_h - 2.0 * top_gap
    door_t = 0.018
    hinge_axis_y = -0.007
    hinge_barrel_r = 0.007
    hinge_body_barrel_len = 0.07
    hinge_door_barrel_len = 0.10
    hinge_barrel_offset_z = door_h / 2.0 - 0.055
    door_panel_w = door_w - hinge_barrel_r

    charcoal = model.material("charcoal", rgba=(0.22, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    painted_panel = model.material("painted_panel", rgba=(0.78, 0.80, 0.82, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.08, 0.09, 0.10, 1.0))

    frame_band_x = (frame_outer_w - opening_w) / 2.0
    frame_band_z = (frame_outer_h - opening_h) / 2.0

    body = model.part("equipment_face")
    body.visual(
        Box((frame_band_x, frame_thickness, frame_outer_h)),
        origin=Origin(xyz=(-opening_w / 2.0 - frame_band_x / 2.0, -frame_thickness / 2.0, 0.0)),
        material=charcoal,
        name="left_frame",
    )
    body.visual(
        Box((frame_band_x, frame_thickness, frame_outer_h)),
        origin=Origin(xyz=(opening_w / 2.0 + frame_band_x / 2.0, -frame_thickness / 2.0, 0.0)),
        material=charcoal,
        name="right_frame",
    )
    body.visual(
        Box((opening_w, frame_thickness, frame_band_z)),
        origin=Origin(xyz=(0.0, -frame_thickness / 2.0, opening_h / 2.0 + frame_band_z / 2.0)),
        material=charcoal,
        name="top_frame",
    )
    body.visual(
        Box((opening_w, frame_thickness, frame_band_z)),
        origin=Origin(xyz=(0.0, -frame_thickness / 2.0, -opening_h / 2.0 - frame_band_z / 2.0)),
        material=charcoal,
        name="bottom_frame",
    )
    body.visual(
        Box((liner_wall, cavity_depth, opening_h)),
        origin=Origin(
            xyz=(-opening_w / 2.0 + liner_wall / 2.0, -frame_thickness - cavity_depth / 2.0, 0.0)
        ),
        material=charcoal,
        name="left_jamb",
    )
    body.visual(
        Box((liner_wall, cavity_depth, opening_h)),
        origin=Origin(
            xyz=(opening_w / 2.0 - liner_wall / 2.0, -frame_thickness - cavity_depth / 2.0, 0.0)
        ),
        material=charcoal,
        name="right_jamb",
    )
    body.visual(
        Box((opening_w - 2.0 * liner_wall, cavity_depth, liner_wall)),
        origin=Origin(
            xyz=(0.0, -frame_thickness - cavity_depth / 2.0, opening_h / 2.0 - liner_wall / 2.0)
        ),
        material=charcoal,
        name="top_jamb",
    )
    body.visual(
        Box((opening_w - 2.0 * liner_wall, cavity_depth, liner_wall)),
        origin=Origin(
            xyz=(0.0, -frame_thickness - cavity_depth / 2.0, -opening_h / 2.0 + liner_wall / 2.0)
        ),
        material=charcoal,
        name="bottom_jamb",
    )
    body.visual(
        Box((opening_w - 2.0 * liner_wall, 0.008, opening_h - 2.0 * liner_wall)),
        origin=Origin(xyz=(0.0, -frame_thickness - cavity_depth + 0.004, 0.0)),
        material=cavity_dark,
        name="recess_back",
    )
    body.visual(
        Box((0.020, 0.004, 0.11)),
        origin=Origin(
            xyz=(-opening_w / 2.0 + side_gap - 0.010, -0.016, hinge_barrel_offset_z)
        ),
        material=steel,
        name="body_hinge_leaf_top",
    )
    body.visual(
        Box((0.020, 0.004, 0.11)),
        origin=Origin(
            xyz=(-opening_w / 2.0 + side_gap - 0.010, -0.016, -hinge_barrel_offset_z)
        ),
        material=steel,
        name="body_hinge_leaf_bottom",
    )
    body.visual(
        Cylinder(radius=hinge_barrel_r, length=hinge_body_barrel_len),
        origin=Origin(
            xyz=(-opening_w / 2.0 + side_gap, hinge_axis_y, hinge_barrel_offset_z)
        ),
        material=steel,
        name="hinge_barrel_top",
    )
    body.visual(
        Cylinder(radius=hinge_barrel_r, length=hinge_body_barrel_len),
        origin=Origin(
            xyz=(-opening_w / 2.0 + side_gap, hinge_axis_y, -hinge_barrel_offset_z)
        ),
        material=steel,
        name="hinge_barrel_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((frame_outer_w, cavity_depth + frame_thickness, frame_outer_h)),
        mass=7.5,
        origin=Origin(xyz=(0.0, -(cavity_depth + frame_thickness) / 2.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_panel_w, door_t, door_h)),
        origin=Origin(xyz=(hinge_barrel_r + door_panel_w / 2.0, -0.009 - hinge_axis_y, 0.0)),
        material=painted_panel,
        name="door_skin",
    )
    door.visual(
        Box((0.024, 0.004, 0.14)),
        origin=Origin(xyz=(0.012, -0.016 - hinge_axis_y, 0.0)),
        material=steel,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=hinge_barrel_r, length=hinge_door_barrel_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel_center",
    )
    door.visual(
        Box((0.030, 0.004, door_h - 0.06)),
        origin=Origin(xyz=(door_w - 0.015, -0.014 - hinge_axis_y, 0.0)),
        material=steel,
        name="latch_stile",
    )
    door.visual(
        Box((0.050, 0.006, 0.085)),
        origin=Origin(xyz=(door_w - 0.032, -0.001 - hinge_axis_y, 0.0)),
        material=steel,
        name="latch_housing",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=3.0,
        origin=Origin(xyz=(door_w / 2.0, -0.009 - hinge_axis_y, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-opening_w / 2.0 + side_gap, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=2.05,
        ),
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

    body = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    ctx.expect_gap(
        body,
        door,
        axis="x",
        positive_elem="right_frame",
        negative_elem="door_skin",
        min_gap=0.0,
        max_gap=0.010,
        name="closed latch edge keeps a narrow frame gap",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    ctx.check(
        "door swings outward from the equipment face",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.20,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
