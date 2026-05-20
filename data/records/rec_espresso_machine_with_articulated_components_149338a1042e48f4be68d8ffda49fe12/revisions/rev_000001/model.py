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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lever_espresso_machine")

    body_metal = model.material("body_metal", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    wood = model.material("wood", rgba=(0.48, 0.30, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.24, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_metal,
        name="base_plate",
    )
    body.visual(
        Box((0.20, 0.20, 0.20)),
        origin=Origin(xyz=(-0.01, 0.0, 0.13)),
        material=body_metal,
        name="pedestal",
    )
    body.visual(
        Cylinder(radius=0.095, length=0.20),
        origin=Origin(xyz=(-0.005, 0.0, 0.275), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_metal,
        name="boiler",
    )
    body.visual(
        Box((0.08, 0.12, 0.09)),
        origin=Origin(xyz=(0.09, 0.0, 0.23)),
        material=body_metal,
        name="front_block",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.150, 0.0, 0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_metal,
        name="group_barrel",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.182, 0.0, 0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_metal,
        name="group_collar",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.036),
        origin=Origin(xyz=(0.165, 0.0, 0.205)),
        material=dark_metal,
        name="group_boss",
    )
    body.visual(
        Box((0.022, 0.012, 0.090)),
        origin=Origin(xyz=(0.115, 0.045, 0.305)),
        material=body_metal,
        name="pivot_cheek_0",
    )
    body.visual(
        Box((0.022, 0.012, 0.090)),
        origin=Origin(xyz=(0.115, -0.045, 0.305)),
        material=body_metal,
        name="pivot_cheek_1",
    )
    body.visual(
        Box((0.060, 0.039, 0.070)),
        origin=Origin(xyz=(0.055, -0.0955, 0.280)),
        material=body_metal,
        name="wand_mount",
    )
    body.visual(
        Box((0.010, 0.126, 0.030)),
        origin=Origin(xyz=(0.071, 0.0, 0.375)),
        material=body_metal,
        name="reservoir_front_wall",
    )
    body.visual(
        Box((0.010, 0.126, 0.030)),
        origin=Origin(xyz=(-0.071, 0.0, 0.375)),
        material=body_metal,
        name="reservoir_rear_wall",
    )
    body.visual(
        Box((0.152, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.058, 0.375)),
        material=body_metal,
        name="reservoir_side_0",
    )
    body.visual(
        Box((0.152, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.058, 0.375)),
        material=body_metal,
        name="reservoir_side_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.26, 0.24, 0.42)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.110, 0.120, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_metal,
        name="tray_bottom",
    )
    drip_tray.visual(
        Box((0.004, 0.120, 0.022)),
        origin=Origin(xyz=(0.053, 0.0, 0.015)),
        material=dark_metal,
        name="tray_front_wall",
    )
    drip_tray.visual(
        Box((0.004, 0.120, 0.022)),
        origin=Origin(xyz=(-0.053, 0.0, 0.015)),
        material=dark_metal,
        name="tray_rear_wall",
    )
    drip_tray.visual(
        Box((0.110, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.058, 0.015)),
        material=dark_metal,
        name="tray_side_0",
    )
    drip_tray.visual(
        Box((0.110, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, -0.058, 0.015)),
        material=dark_metal,
        name="tray_side_1",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.112, 0.122, 0.024)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.012, length=0.068),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.012, length=0.310),
        origin=Origin(xyz=(-0.018, 0.0, 0.155)),
        material=dark_metal,
        name="lever_arm",
    )
    lever.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(-0.022, 0.0, 0.322)),
        material=wood,
        name="lever_grip",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.060, 0.070, 0.350)),
        mass=0.55,
        origin=Origin(xyz=(-0.015, 0.0, 0.175)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.039, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_metal,
        name="pf_flange",
    )
    portafilter.visual(
        Cylinder(radius=0.032, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=body_metal,
        name="pf_basket",
    )
    portafilter.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.020, 0.028, -0.001)),
        material=dark_metal,
        name="pf_lug_0",
    )
    portafilter.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.020, -0.028, -0.001)),
        material=dark_metal,
        name="pf_lug_1",
    )
    portafilter.visual(
        Box((0.040, 0.016, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, -0.007)),
        material=dark_metal,
        name="handle_neck",
    )
    portafilter.visual(
        Cylinder(radius=0.013, length=0.140),
        origin=Origin(xyz=(0.118, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle_grip",
    )
    portafilter.visual(
        Box((0.020, 0.024, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, -0.040)),
        material=dark_metal,
        name="spout_block",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.220, 0.090, 0.060)),
        mass=0.45,
        origin=Origin(xyz=(0.090, 0.0, -0.018)),
    )

    steam_wand = model.part("steam_wand")
    wand_tube = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.018, 0.0, -0.004),
            (0.040, 0.0, -0.012),
            (0.048, 0.0, -0.120),
        ],
        radius=0.005,
        samples_per_segment=18,
        radial_segments=16,
    )
    steam_wand.visual(
        mesh_from_geometry(wand_tube, "steam_wand_tube"),
        material=body_metal,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_metal,
        name="wand_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.048, 0.0, -0.129)),
        material=body_metal,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.070, 0.020, 0.150)),
        mass=0.18,
        origin=Origin(xyz=(0.024, 0.0, -0.070)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.145, 0.118, 0.006)),
        origin=Origin(xyz=(0.0725, 0.0, 0.003)),
        material=body_metal,
        name="lid_panel",
    )
    lid.visual(
        Box((0.020, 0.050, 0.010)),
        origin=Origin(xyz=(0.137, 0.0, 0.008)),
        material=dark_metal,
        name="lid_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.145, 0.118, 0.018)),
        mass=0.20,
        origin=Origin(xyz=(0.073, 0.0, 0.009)),
    )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.FIXED,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.065, 0.0, 0.030)),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.115, 0.0, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=-1.20,
            upper=0.15,
        ),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.165, 0.0, 0.187)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-1.05,
            upper=0.0,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.083, -0.124, 0.295)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=-1.20,
            upper=0.80,
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.071, 0.0, 0.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
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

    body = object_model.get_part("body")
    drip_tray = object_model.get_part("drip_tray")
    lever = object_model.get_part("lever")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    lid = object_model.get_part("lid")

    lever_joint = object_model.get_articulation("body_to_lever")
    pf_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    lid_joint = object_model.get_articulation("body_to_lid")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.expect_gap(
        drip_tray,
        body,
        axis="z",
        positive_elem="tray_bottom",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="drip tray sits on the base plate",
    )
    ctx.expect_overlap(
        drip_tray,
        body,
        axes="xy",
        elem_a="tray_bottom",
        elem_b="base_plate",
        min_overlap=0.10,
        name="drip tray stays within the base footprint",
    )
    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_boss",
        negative_elem="pf_flange",
        max_gap=0.001,
        max_penetration=0.0,
        name="portafilter flange seats against the group head",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="reservoir_front_wall",
        max_gap=0.002,
        max_penetration=0.0,
        name="lid closes onto the reservoir rim",
    )

    rest_lever = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_grip"))
    with ctx.pose({lever_joint: -1.0}):
        pulled_lever = aabb_center(ctx.part_element_world_aabb(lever, elem="lever_grip"))
    ctx.check(
        "lever pulls forward and downward",
        rest_lever is not None
        and pulled_lever is not None
        and pulled_lever[0] > rest_lever[0] + 0.08
        and pulled_lever[2] < rest_lever[2] - 0.12,
        details=f"rest={rest_lever}, pulled={pulled_lever}",
    )

    locked_pf = aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle_grip"))
    with ctx.pose({pf_joint: -1.0}):
        insert_pf = aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle_grip"))
    ctx.check(
        "portafilter rotates away from the front lock position",
        locked_pf is not None
        and insert_pf is not None
        and abs(insert_pf[1] - locked_pf[1]) > 0.09
        and insert_pf[0] < locked_pf[0] - 0.03,
        details=f"locked={locked_pf}, insert={insert_pf}",
    )

    parked_wand = aabb_center(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    with ctx.pose({wand_joint: 0.8}):
        swung_wand = aabb_center(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    ctx.check(
        "steam wand swings laterally on its support pivot",
        parked_wand is not None
        and swung_wand is not None
        and swung_wand[1] > parked_wand[1] + 0.03,
        details=f"parked={parked_wand}, swung={swung_wand}",
    )

    closed_lid = aabb_center(ctx.part_element_world_aabb(lid, elem="lid_panel"))
    with ctx.pose({lid_joint: 1.2}):
        open_lid = aabb_center(ctx.part_element_world_aabb(lid, elem="lid_panel"))
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid is not None
        and open_lid is not None
        and open_lid[2] > closed_lid[2] + 0.05
        and open_lid[0] < closed_lid[0] - 0.03,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
