from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in (
        {"name": name, "rgba": rgba},
        {"name": name, "color": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    material = Material(name=name)
    for attr in ("rgba", "color"):
        try:
            setattr(material, attr, rgba)
            break
        except Exception:
            continue
    return material


def _origin(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(xyz=xyz, rpy=rpy)


def _box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    *,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=_origin(xyz, rpy), material=material, name=name)


def _cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    *,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(xyz, rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refrigerator", assets=ASSETS)

    materials = {
        "enamel_white": _make_material("enamel_white", (0.96, 0.97, 0.98, 1.0)),
        "warm_gray_plastic": _make_material("warm_gray_plastic", (0.84, 0.86, 0.88, 1.0)),
        "dark_rubber": _make_material("dark_rubber", (0.16, 0.17, 0.19, 1.0)),
        "smoked_black": _make_material("smoked_black", (0.10, 0.11, 0.12, 1.0)),
        "stainless": _make_material("stainless", (0.74, 0.76, 0.79, 1.0)),
        "glass": _make_material("glass", (0.80, 0.90, 0.98, 0.35)),
        "tray_clear": _make_material("tray_clear", (0.88, 0.93, 0.98, 0.55)),
    }
    if isinstance(getattr(model, "materials", None), list):
        model.materials.extend(materials.values())

    width = 0.92
    depth = 0.74
    height = 1.82
    wall_t = 0.03
    top_t = 0.03
    floor_t = 0.03
    plinth_h = 0.08
    freezer_open_h = 0.44
    upper_base_z = plinth_h + freezer_open_h + floor_t
    upper_open_h = height - top_t - upper_base_z

    hinge_inset = 0.008
    center_gap = 0.012
    door_t = 0.062
    door_w = width / 2.0 - hinge_inset - center_gap / 2.0
    door_h = upper_open_h - 0.009
    door_bottom_z = upper_base_z + 0.0045
    door_hinge_y = depth / 2.0 - 0.004
    door_hinge_z = door_bottom_z + door_h / 2.0
    door_panel_y = 0.008 + door_t / 2.0

    drawer_front_t = 0.074
    drawer_bottom_z = plinth_h + 0.004
    drawer_top_z = door_bottom_z - 0.006
    drawer_front_h = drawer_top_z - drawer_bottom_z
    drawer_joint_y = depth / 2.0 - 0.008
    drawer_joint_z = plinth_h + floor_t / 2.0
    drawer_front_cz = (drawer_bottom_z + drawer_top_z) / 2.0 - drawer_joint_z
    drawer_front_cy = 0.008 + drawer_front_t / 2.0
    drawer_bin_w = width - 0.120

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=92.0,
        origin=_origin((0.0, 0.0, height / 2.0)),
    )

    _box(
        cabinet,
        (wall_t, depth, height),
        (-width / 2.0 + wall_t / 2.0, 0.0, height / 2.0),
        materials["enamel_white"],
        name="left_body_wall",
    )
    _box(
        cabinet,
        (wall_t, depth, height),
        (width / 2.0 - wall_t / 2.0, 0.0, height / 2.0),
        materials["enamel_white"],
        name="right_body_wall",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t, depth, top_t),
        (0.0, 0.0, height - top_t / 2.0),
        materials["enamel_white"],
        name="top_cap",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t, depth, floor_t),
        (0.0, 0.0, plinth_h + floor_t / 2.0),
        materials["warm_gray_plastic"],
        name="bottom_deck",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t, depth, floor_t),
        (0.0, 0.0, plinth_h + freezer_open_h + floor_t / 2.0),
        materials["warm_gray_plastic"],
        name="freezer_ceiling",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t, wall_t, height),
        (0.0, -depth / 2.0 + wall_t / 2.0, height / 2.0),
        materials["warm_gray_plastic"],
        name="back_liner",
    )
    _box(
        cabinet,
        (width - 0.08, 0.10, plinth_h),
        (0.0, depth / 2.0 - 0.13, plinth_h / 2.0),
        materials["dark_rubber"],
        name="toe_kick",
    )
    _box(
        cabinet,
        (0.40, 0.008, 0.030),
        (0.0, depth / 2.0 - 0.082, 0.038),
        materials["smoked_black"],
        name="toe_kick_grille",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t + 0.002, 0.50, 0.008),
        (0.0, -0.12, 0.78),
        materials["glass"],
        name="lower_glass_shelf",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t + 0.002, 0.46, 0.008),
        (0.0, -0.14, 1.10),
        materials["glass"],
        name="mid_glass_shelf",
    )
    _box(
        cabinet,
        (width - 2.0 * wall_t + 0.002, 0.40, 0.008),
        (0.0, -0.17, 1.42),
        materials["glass"],
        name="upper_glass_shelf",
    )
    _box(
        cabinet,
        (0.32, 0.10, 0.030),
        (0.0, 0.12, height - top_t - 0.015),
        materials["warm_gray_plastic"],
        name="ceiling_control_housing",
    )
    _box(
        cabinet,
        (0.12, 0.004, 0.018),
        (0.0, 0.162, height - top_t - 0.017),
        materials["smoked_black"],
        name="ceiling_display_strip",
    )
    _box(
        cabinet,
        (0.05, 0.045, 0.026),
        (-width / 2.0 + 0.040, depth / 2.0 - 0.022, height - 0.013),
        materials["warm_gray_plastic"],
        name="left_hinge_cover",
    )
    _box(
        cabinet,
        (0.05, 0.045, 0.026),
        (width / 2.0 - 0.040, depth / 2.0 - 0.022, height - 0.013),
        materials["warm_gray_plastic"],
        name="right_hinge_cover",
    )

    def add_door(
        name: str, hinge_x: float, axis: tuple[float, float, float], is_left: bool
    ) -> None:
        door = model.part(name)
        door.inertial = Inertial.from_geometry(
            Box((door_w, door_t, door_h)),
            mass=17.0,
            origin=_origin(((door_w / 2.0 if is_left else -door_w / 2.0), door_panel_y, 0.0)),
        )

        sign = 1.0 if is_left else -1.0

        def sx(u: float) -> float:
            return sign * u

        x_mid = sx(door_w / 2.0)
        _box(
            door,
            (door_w, door_t, door_h),
            (x_mid, door_panel_y, 0.0),
            materials["enamel_white"],
            name=f"{name}_outer_panel",
        )
        _box(
            door,
            (door_w - 0.050, 0.018, door_h - 0.050),
            (x_mid, 0.017, 0.0),
            materials["warm_gray_plastic"],
            name=f"{name}_inner_liner",
        )
        _box(
            door,
            (door_w - 0.030, 0.008, 0.015),
            (x_mid, 0.010, door_h / 2.0 - 0.012),
            materials["dark_rubber"],
            name=f"{name}_top_gasket",
        )
        _box(
            door,
            (door_w - 0.030, 0.008, 0.015),
            (x_mid, 0.010, -door_h / 2.0 + 0.012),
            materials["dark_rubber"],
            name=f"{name}_bottom_gasket",
        )
        _box(
            door,
            (0.015, 0.008, door_h - 0.024),
            (sx(0.013), 0.010, 0.0),
            materials["dark_rubber"],
            name=f"{name}_hinge_side_gasket",
        )
        _box(
            door,
            (0.015, 0.008, door_h - 0.024),
            (sx(door_w - 0.013), 0.010, 0.0),
            materials["dark_rubber"],
            name=f"{name}_free_side_gasket",
        )

        bin_specs = [
            (-0.33, 0.090, door_w - 0.130, 0.086),
            (-0.05, 0.105, door_w - 0.110, 0.090),
            (0.23, 0.115, door_w - 0.095, 0.094),
        ]
        for idx, (zc, bh, bw, bd) in enumerate(bin_specs, start=1):
            _box(
                door,
                (bw, 0.010, bh),
                (x_mid, 0.003, zc),
                materials["warm_gray_plastic"],
                name=f"{name}_bin_{idx}_back",
            )
            _box(
                door,
                (bw - 0.020, bd - 0.012, 0.008),
                (x_mid, -0.036, zc - bh / 2.0 + 0.004),
                materials["warm_gray_plastic"],
                name=f"{name}_bin_{idx}_floor",
            )
            _box(
                door,
                (0.012, bd, bh),
                (sx((door_w - bw) / 2.0 + 0.006), -0.037, zc),
                materials["warm_gray_plastic"],
                name=f"{name}_bin_{idx}_hinge_side",
            )
            _box(
                door,
                (0.012, bd, bh),
                (sx(door_w - (door_w - bw) / 2.0 - 0.006), -0.037, zc),
                materials["warm_gray_plastic"],
                name=f"{name}_bin_{idx}_free_side",
            )
            _box(
                door,
                (bw - 0.028, 0.010, bh * 0.42),
                (x_mid, -0.078, zc - bh * 0.14),
                materials["tray_clear"],
                name=f"{name}_bin_{idx}_front_rail",
            )

        handle_x = sx(door_w - 0.055)
        handle_y = door_panel_y + door_t / 2.0 + 0.024
        handle_z = -0.03
        _cylinder(
            door,
            0.010,
            0.78,
            (handle_x, handle_y, handle_z),
            materials["stainless"],
            name=f"{name}_handle_grip",
        )
        for idx, mount_z in enumerate((handle_z - 0.24, handle_z + 0.24), start=1):
            _cylinder(
                door,
                0.007,
                0.048,
                (handle_x, door_panel_y + door_t / 2.0 + 0.010, mount_z),
                materials["stainless"],
                name=f"{name}_handle_standoff_{idx}",
                rpy=(math.pi / 2.0, 0.0, 0.0),
            )
            _box(
                door,
                (0.028, 0.004, 0.050),
                (handle_x, door_panel_y + door_t / 2.0 + 0.002, mount_z),
                materials["stainless"],
                name=f"{name}_handle_mount_{idx}",
            )

        if is_left:
            disp_x = sx(door_w * 0.54)
            disp_z = -0.08
            _box(
                door,
                (0.130, 0.012, 0.250),
                (disp_x, door_panel_y + door_t / 2.0 - 0.010, disp_z),
                materials["smoked_black"],
                name="left_door_dispenser_pocket",
            )
            _box(
                door,
                (0.150, 0.006, 0.014),
                (disp_x, door_panel_y + door_t / 2.0 + 0.003, disp_z + 0.132),
                materials["stainless"],
                name="left_door_dispenser_top_trim",
            )
            _box(
                door,
                (0.150, 0.006, 0.014),
                (disp_x, door_panel_y + door_t / 2.0 + 0.003, disp_z - 0.132),
                materials["stainless"],
                name="left_door_dispenser_bottom_trim",
            )
            _box(
                door,
                (0.014, 0.006, 0.250),
                (disp_x - 0.068, door_panel_y + door_t / 2.0 + 0.003, disp_z),
                materials["stainless"],
                name="left_door_dispenser_left_trim",
            )
            _box(
                door,
                (0.014, 0.006, 0.250),
                (disp_x + 0.068, door_panel_y + door_t / 2.0 + 0.003, disp_z),
                materials["stainless"],
                name="left_door_dispenser_right_trim",
            )
            _box(
                door,
                (0.080, 0.022, 0.012),
                (disp_x, door_panel_y + door_t / 2.0 + 0.010, disp_z - 0.095),
                materials["stainless"],
                name="left_door_drip_tray",
            )
            _box(
                door,
                (0.036, 0.014, 0.090),
                (disp_x, door_panel_y + door_t / 2.0 - 0.004, disp_z - 0.005),
                materials["dark_rubber"],
                name="left_door_dispense_paddle",
            )
            _box(
                door,
                (0.074, 0.012, 0.028),
                (disp_x, door_panel_y + door_t / 2.0 - 0.002, disp_z + 0.090),
                materials["smoked_black"],
                name="left_door_display_panel",
            )

        model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent="cabinet",
            child=name,
            origin=_origin((hinge_x, door_hinge_y, door_hinge_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=1.6,
                lower=0.0,
                upper=1.55,
            ),
        )

    add_door("left_door", -width / 2.0 + hinge_inset, (0.0, 0.0, 1.0), True)
    add_door("right_door", width / 2.0 - hinge_inset, (0.0, 0.0, -1.0), False)

    freezer = model.part("freezer_drawer")
    freezer.inertial = Inertial.from_geometry(
        Box((width - 0.03, drawer_front_t, drawer_front_h)),
        mass=21.0,
        origin=_origin((0.0, drawer_front_cy, drawer_front_cz)),
    )
    _box(
        freezer,
        (width - 0.028, drawer_front_t, drawer_front_h),
        (0.0, drawer_front_cy, drawer_front_cz),
        materials["enamel_white"],
        name="drawer_front_panel",
    )
    _box(
        freezer,
        (width - 0.055, 0.010, 0.014),
        (0.0, 0.010, drawer_front_cz + drawer_front_h / 2.0 - 0.012),
        materials["dark_rubber"],
        name="drawer_top_gasket",
    )
    _box(
        freezer,
        (drawer_bin_w, 0.50, 0.012),
        (0.0, -0.200, 0.040),
        materials["warm_gray_plastic"],
        name="drawer_bin_floor",
    )
    _box(
        freezer,
        (drawer_bin_w, 0.012, 0.230),
        (0.0, -0.444, 0.156),
        materials["warm_gray_plastic"],
        name="drawer_bin_back",
    )
    _box(
        freezer,
        (0.012, 0.50, 0.230),
        (-drawer_bin_w / 2.0, -0.200, 0.156),
        materials["warm_gray_plastic"],
        name="drawer_bin_left_wall",
    )
    _box(
        freezer,
        (0.012, 0.50, 0.230),
        (drawer_bin_w / 2.0, -0.200, 0.156),
        materials["warm_gray_plastic"],
        name="drawer_bin_right_wall",
    )
    _box(
        freezer,
        (drawer_bin_w - 0.050, 0.010, 0.120),
        (0.0, -0.438, 0.182),
        materials["tray_clear"],
        name="drawer_inner_basket_front",
    )
    _cylinder(
        freezer,
        0.010,
        0.46,
        (0.0, drawer_front_cy + drawer_front_t / 2.0 + 0.026, drawer_front_cz),
        materials["stainless"],
        name="drawer_handle_grip",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    for idx, mount_x in enumerate((-0.15, 0.15), start=1):
        _cylinder(
            freezer,
            0.007,
            0.050,
            (mount_x, drawer_front_cy + drawer_front_t / 2.0 + 0.011, drawer_front_cz),
            materials["stainless"],
            name=f"drawer_handle_standoff_{idx}",
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        _box(
            freezer,
            (0.040, 0.004, 0.028),
            (mount_x, drawer_front_cy + drawer_front_t / 2.0 + 0.002, drawer_front_cz),
            materials["stainless"],
            name=f"drawer_handle_mount_{idx}",
        )

    model.articulation(
        "freezer_drawer_slide",
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child="freezer_drawer",
        origin=_origin((0.0, drawer_joint_y, drawer_joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=0.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "cabinet",
        "freezer_drawer",
        reason="closed freezer drawer nests against the cabinet opening with a compressible gasket, producing a conservative contact overlap",
    )
    ctx.allow_overlap(
        "cabinet",
        "left_door",
        reason="hinge-side sweep stays tight to the cabinet knuckle at full open",
    )
    ctx.allow_overlap(
        "cabinet",
        "right_door",
        reason="hinge-side sweep stays tight to the cabinet knuckle at full open",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "left_door_hinge",
        "left_door",
        world_axis="y",
        direction="positive",
        min_delta=0.12,
    )
    ctx.expect_joint_motion_axis(
        "right_door_hinge",
        "right_door",
        world_axis="y",
        direction="positive",
        min_delta=0.12,
    )
    ctx.expect_joint_motion_axis(
        "freezer_drawer_slide",
        "freezer_drawer",
        world_axis="y",
        direction="positive",
        min_delta=0.18,
    )

    ctx.expect_aabb_gap("left_door", "freezer_drawer", axis="z", max_gap=0.012, max_penetration=0.0)
    ctx.expect_aabb_gap("right_door", "freezer_drawer", axis="z", max_gap=0.012, max_penetration=0.0)
    ctx.expect_aabb_overlap("left_door", "freezer_drawer", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_overlap("right_door", "freezer_drawer", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_overlap("freezer_drawer", "cabinet", axes="xy", min_overlap=0.20)

    with ctx.pose(left_door_hinge=1.35):
        ctx.expect_aabb_gap("left_door", "freezer_drawer", axis="z", max_gap=0.012, max_penetration=0.0)
        ctx.expect_aabb_overlap("right_door", "freezer_drawer", axes="xy", min_overlap=0.04)

    with ctx.pose(right_door_hinge=1.35):
        ctx.expect_aabb_gap("right_door", "freezer_drawer", axis="z", max_gap=0.012, max_penetration=0.0)
        ctx.expect_aabb_overlap("left_door", "freezer_drawer", axes="xy", min_overlap=0.04)

    with ctx.pose(freezer_drawer_slide=0.34):
        ctx.expect_aabb_gap("left_door", "freezer_drawer", axis="z", max_gap=0.012, max_penetration=0.0)
        ctx.expect_aabb_gap("right_door", "freezer_drawer", axis="z", max_gap=0.012, max_penetration=0.0)
        ctx.expect_aabb_overlap("freezer_drawer", "cabinet", axes="xy", min_overlap=0.10)

    with ctx.pose(left_door_hinge=1.20, right_door_hinge=1.20, freezer_drawer_slide=0.22):
        ctx.expect_aabb_gap("left_door", "freezer_drawer", axis="z", max_gap=0.016, max_penetration=0.0)
        ctx.expect_aabb_gap("right_door", "freezer_drawer", axis="z", max_gap=0.016, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
