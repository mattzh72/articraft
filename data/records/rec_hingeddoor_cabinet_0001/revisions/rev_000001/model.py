from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _make_material(name: str, rgba: tuple[float, float, float, float]):
    for kwargs in (
        {"name": name, "rgba": rgba},
        {"name": name, "color": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            continue
    return None


def _add_box(part, size, xyz, material=None, name: str | None = None) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, radius, length, xyz, material=None, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _world_aabb_bounds(
    ctx: TestContext, part_name: str, use: str = "collision"
) -> tuple[float, float, float, float, float, float]:
    aabb = ctx.part_world_aabb(part_name, use=use)

    attr_layouts = (
        ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z"),
        ("xmin", "ymin", "zmin", "xmax", "ymax", "zmax"),
    )
    for names in attr_layouts:
        if all(hasattr(aabb, name) for name in names):
            return tuple(float(getattr(aabb, name)) for name in names)

    pair_layouts = (("mins", "maxs"), ("min", "max"), ("minimum", "maximum"), ("lower", "upper"))
    for min_name, max_name in pair_layouts:
        if hasattr(aabb, min_name) and hasattr(aabb, max_name):
            mins = getattr(aabb, min_name)
            maxs = getattr(aabb, max_name)
            if len(mins) == 3 and len(maxs) == 3:
                return (
                    float(mins[0]),
                    float(mins[1]),
                    float(mins[2]),
                    float(maxs[0]),
                    float(maxs[1]),
                    float(maxs[2]),
                )

    if isinstance(aabb, dict):
        if all(key in aabb for key in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")):
            return (
                float(aabb["min_x"]),
                float(aabb["min_y"]),
                float(aabb["min_z"]),
                float(aabb["max_x"]),
                float(aabb["max_y"]),
                float(aabb["max_z"]),
            )
        if all(key in aabb for key in ("mins", "maxs")):
            mins = aabb["mins"]
            maxs = aabb["maxs"]
            return (
                float(mins[0]),
                float(mins[1]),
                float(mins[2]),
                float(maxs[0]),
                float(maxs[1]),
                float(maxs[2]),
            )

    if (
        isinstance(aabb, (tuple, list))
        and len(aabb) == 2
        and len(aabb[0]) == 3
        and len(aabb[1]) == 3
    ):
        mins, maxs = aabb
        return (
            float(mins[0]),
            float(mins[1]),
            float(mins[2]),
            float(maxs[0]),
            float(maxs[1]),
            float(maxs[2]),
        )

    raise TypeError(f"Unsupported AABB representation for part {part_name!r}: {type(aabb)!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="furniture_cabinet", assets=ASSETS)

    walnut = _make_material("walnut_veneer", (0.42, 0.29, 0.19, 1.0))
    oak_inner = _make_material("oiled_oak", (0.63, 0.52, 0.36, 1.0))
    steel = _make_material("brushed_steel", (0.72, 0.73, 0.74, 1.0))
    dark_metal = _make_material("dark_iron", (0.20, 0.20, 0.22, 1.0))
    model.materials.extend([m for m in (walnut, oak_inner, steel, dark_metal) if m is not None])

    width = 1.20
    depth = 0.42
    height = 0.96
    side_t = 0.022
    top_t = 0.030
    back_t = 0.010
    plinth_h = 0.080

    side_h = height - plinth_h - top_t
    side_z = plinth_h + side_h / 2.0
    top_z = height - top_t / 2.0
    bottom_z = plinth_h + side_t / 2.0
    back_h = height - plinth_h - top_t - side_t
    back_z = plinth_h + side_t + back_h / 2.0

    pull_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.000, -0.090),
                (0.0, 0.012, -0.050),
                (0.0, 0.018, 0.050),
                (0.0, 0.000, 0.090),
            ],
            radius=0.004,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("cabinet_pull.obj"),
    )

    body = model.part("cabinet_body")
    _add_box(
        body,
        (side_t, depth, side_h),
        (-width / 2.0 + side_t / 2.0, 0.0, side_z),
        walnut,
        "left_side",
    )
    _add_box(
        body,
        (side_t, depth, side_h),
        (width / 2.0 - side_t / 2.0, 0.0, side_z),
        walnut,
        "right_side",
    )
    _add_box(body, (width + 0.016, depth, top_t), (0.0, 0.0, top_z), walnut, "top_cap")
    _add_box(
        body, (width - 2.0 * side_t, depth, side_t), (0.0, 0.0, bottom_z), oak_inner, "bottom_panel"
    )
    _add_box(
        body,
        (width - 2.0 * side_t, back_t, back_h),
        (0.0, -depth / 2.0 + back_t / 2.0, back_z),
        oak_inner,
        "back_panel",
    )
    _add_box(
        body,
        (width - 2.0 * side_t, 0.026, 0.060),
        (0.0, depth / 2.0 - 0.013, 0.900),
        walnut,
        "top_face_rail",
    )
    _add_box(
        body,
        (width - 2.0 * side_t, 0.026, 0.055),
        (0.0, depth / 2.0 - 0.013, 0.1075),
        walnut,
        "bottom_face_rail",
    )
    _add_box(
        body,
        (width - 2.0 * side_t, depth - 0.020, 0.022),
        (0.0, 0.0, 0.500),
        oak_inner,
        "interior_shelf",
    )

    hinge_axis_y = depth / 2.0 - 0.002
    left_axis_x = -0.5985
    right_axis_x = 0.5985
    for hinge_z in (0.280, 0.740):
        _add_box(
            body,
            (0.004, 0.050, 0.095),
            (-0.602, depth / 2.0 - 0.024, hinge_z),
            dark_metal,
            f"left_hinge_leaf_{hinge_z:.3f}",
        )
        _add_cylinder(
            body,
            0.005,
            0.095,
            (left_axis_x, hinge_axis_y, hinge_z),
            dark_metal,
            f"left_hinge_barrel_{hinge_z:.3f}",
        )
        _add_box(
            body,
            (0.004, 0.050, 0.095),
            (0.602, depth / 2.0 - 0.024, hinge_z),
            dark_metal,
            f"right_hinge_leaf_{hinge_z:.3f}",
        )
        _add_cylinder(
            body,
            0.005,
            0.095,
            (right_axis_x, hinge_axis_y, hinge_z),
            dark_metal,
            f"right_hinge_barrel_{hinge_z:.3f}",
        )

    body.inertial = Inertial.from_geometry(
        Box((width + 0.016, depth, height - plinth_h)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, (plinth_h + height) / 2.0)),
    )

    plinth = model.part("plinth")
    _add_box(
        plinth,
        (width - 0.080, 0.018, plinth_h),
        (0.0, depth / 2.0 - 0.049, -plinth_h / 2.0),
        walnut,
        "front_kick",
    )
    _add_box(
        plinth,
        (0.018, depth - 0.080, plinth_h),
        (-width / 2.0 + 0.039, 0.0, -plinth_h / 2.0),
        walnut,
        "left_return",
    )
    _add_box(
        plinth,
        (0.018, depth - 0.080, plinth_h),
        (width / 2.0 - 0.039, 0.0, -plinth_h / 2.0),
        walnut,
        "right_return",
    )
    _add_box(
        plinth,
        (0.050, depth - 0.080, plinth_h),
        (0.0, 0.0, -plinth_h / 2.0),
        oak_inner,
        "center_support",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((width - 0.080, depth - 0.080, plinth_h)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -plinth_h / 2.0)),
    )

    door_width = (width - 0.008 - 0.003) / 2.0
    door_height = 0.830
    door_t = 0.022
    door_center_y = 0.015
    edge_clearance = 0.0025
    panel_center_x = edge_clearance + door_width / 2.0
    stile_w = 0.070
    rail_h = 0.080
    trim_t = 0.004
    trim_y = door_center_y + door_t / 2.0 + trim_t / 2.0 - 0.0005
    panel_y = door_center_y + 0.004
    hinge_leaf_x = 0.0085
    hinge_leaf_y = door_center_y + 0.009
    handle_x = edge_clearance + door_width - 0.075

    def add_door(part_name: str, sign: float) -> None:
        door = model.part(part_name)
        _add_box(
            door,
            (door_width, door_t, door_height),
            (sign * panel_center_x, door_center_y, 0.0),
            walnut,
            "door_slab",
        )
        _add_box(
            door,
            (stile_w, trim_t, door_height - 0.055),
            (sign * (edge_clearance + stile_w / 2.0), trim_y, 0.0),
            walnut,
            "outer_stile",
        )
        _add_box(
            door,
            (stile_w, trim_t, door_height - 0.055),
            (sign * (edge_clearance + door_width - stile_w / 2.0), trim_y, 0.0),
            walnut,
            "inner_stile",
        )
        _add_box(
            door,
            (door_width - 2.0 * stile_w + 0.006, trim_t, rail_h),
            (sign * panel_center_x, trim_y, door_height / 2.0 - rail_h / 2.0),
            walnut,
            "top_rail",
        )
        _add_box(
            door,
            (door_width - 2.0 * stile_w + 0.006, trim_t, rail_h),
            (sign * panel_center_x, trim_y, -door_height / 2.0 + rail_h / 2.0),
            walnut,
            "bottom_rail",
        )
        _add_box(
            door,
            (door_width - 0.180, 0.012, door_height - 0.200),
            (sign * panel_center_x, panel_y, 0.0),
            oak_inner,
            "center_panel",
        )
        for hinge_z_local in (-0.230, 0.230):
            _add_box(
                door,
                (0.012, 0.028, 0.095),
                (sign * hinge_leaf_x, hinge_leaf_y, hinge_z_local),
                dark_metal,
                f"hinge_leaf_{hinge_z_local:+.3f}",
            )
        door.visual(
            pull_mesh,
            origin=Origin(xyz=(sign * handle_x, door_center_y + door_t / 2.0, 0.0)),
            material=steel,
            name="pull_handle",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_width, door_t, door_height)),
            mass=7.5,
            origin=Origin(xyz=(sign * panel_center_x, door_center_y, 0.0)),
        )

    add_door("left_door", 1.0)
    add_door("right_door", -1.0)

    model.articulation(
        "body_to_plinth",
        ArticulationType.FIXED,
        parent="cabinet_body",
        child="plinth",
        origin=Origin(xyz=(0.0, 0.0, plinth_h)),
    )
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet_body",
        child="left_door",
        origin=Origin(xyz=(left_axis_x, hinge_axis_y, 0.510)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.70),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet_body",
        child="right_door",
        origin=Origin(xyz=(right_axis_x, hinge_axis_y, 0.510)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_within("plinth", "cabinet_body", axes="xy")
    ctx.expect_origin_distance("plinth", "cabinet_body", axes="xy", max_dist=0.03)
    ctx.expect_aabb_gap("cabinet_body", "plinth", axis="z", max_gap=0.003, max_penetration=0.0)

    body_min_x, body_min_y, _, body_max_x, body_max_y, body_max_z = _world_aabb_bounds(
        ctx, "cabinet_body"
    )
    left_min_x, _, left_min_z, left_max_x, left_max_y, left_max_z = _world_aabb_bounds(
        ctx, "left_door"
    )
    right_min_x, _, right_min_z, right_max_x, right_max_y, right_max_z = _world_aabb_bounds(
        ctx, "right_door"
    )

    assert body_max_z > 0.94, "cabinet body should reach full furniture height"
    assert 0.012 <= left_max_y - body_max_y <= 0.05, (
        "left door and handle should project slightly beyond the cabinet face when closed"
    )
    assert 0.012 <= right_max_y - body_max_y <= 0.05, (
        "right door and handle should project slightly beyond the cabinet face when closed"
    )
    assert abs(left_max_z - right_max_z) <= 0.002 and abs(left_min_z - right_min_z) <= 0.002, (
        "doors should align vertically"
    )
    assert -0.01 <= left_max_x <= 0.0, "left door should close to the center seam"
    assert 0.0 <= right_min_x <= 0.01, "right door should close to the center seam"
    assert 0.0 <= right_min_x - left_max_x <= 0.006, (
        "center reveal between the two doors should stay tight"
    )

    ctx.expect_joint_motion_axis(
        "left_door_hinge", "left_door", world_axis="y", direction="positive", min_delta=0.10
    )
    ctx.expect_joint_motion_axis(
        "right_door_hinge", "right_door", world_axis="y", direction="positive", min_delta=0.10
    )

    with ctx.pose(left_door_hinge=1.45):
        open_left_min_x, _, _, open_left_max_x, open_left_max_y, _ = _world_aabb_bounds(
            ctx, "left_door"
        )
        assert open_left_max_y >= body_max_y + 0.26, (
            "left door should swing clearly out in front when opened"
        )
        assert open_left_min_x <= body_min_x + 0.02, (
            "left door should keep its hinged edge anchored near the left-side casework"
        )
        assert open_left_max_x <= -0.46, (
            "left door should remain swung toward the left side when opened"
        )
    with ctx.pose(right_door_hinge=1.45):
        open_right_min_x, _, _, open_right_max_x, open_right_max_y, _ = _world_aabb_bounds(
            ctx, "right_door"
        )
        assert open_right_max_y >= body_max_y + 0.26, (
            "right door should swing clearly out in front when opened"
        )
        assert open_right_max_x >= body_max_x - 0.02, (
            "right door should keep its hinged edge anchored near the right-side casework"
        )
        assert open_right_min_x >= 0.46, (
            "right door should remain swung toward the right side when opened"
        )
    with ctx.pose(left_door_hinge=1.45, right_door_hinge=1.45):
        _, _, _, both_left_max_x, both_left_max_y, _ = _world_aabb_bounds(ctx, "left_door")
        both_right_min_x, _, _, _, both_right_max_y, _ = _world_aabb_bounds(ctx, "right_door")
        assert both_left_max_y >= body_max_y + 0.26 and both_right_max_y >= body_max_y + 0.26, (
            "both doors should clear the front opening together"
        )
        assert both_right_min_x - both_left_max_x >= 1.05, (
            "opened doors should create a wide accessible cabinet opening"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
