from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    housing_steel = model.material("housing_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    band_steel = model.material("band_steel", rgba=(0.47, 0.50, 0.54, 1.0))
    flap_steel = model.material("flap_steel", rgba=(0.56, 0.59, 0.62, 1.0))

    shell = model.part("main_shell")
    shell.visual(
        Box((0.52, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=housing_steel,
        name="base_curb",
    )
    shell.visual(
        Box((0.02, 0.30, 0.78)),
        origin=Origin(xyz=(-0.17, 0.0, 0.43)),
        material=housing_steel,
        name="left_wall",
    )
    shell.visual(
        Box((0.02, 0.30, 0.78)),
        origin=Origin(xyz=(0.17, 0.0, 0.43)),
        material=housing_steel,
        name="right_wall",
    )
    shell.visual(
        Box((0.36, 0.02, 0.78)),
        origin=Origin(xyz=(0.0, -0.15, 0.43)),
        material=housing_steel,
        name="back_wall",
    )
    shell.visual(
        Box((0.36, 0.02, 0.40)),
        origin=Origin(xyz=(0.0, 0.15, 0.24)),
        material=housing_steel,
        name="front_lower_wall",
    )
    shell.visual(
        Box((0.36, 0.34, 0.02)),
        origin=Origin(xyz=(0.0, -0.01, 0.82)),
        material=housing_steel,
        name="roof_cap",
    )
    shell.visual(
        Box((0.36, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.17, 0.445)),
        material=housing_steel,
        name="outlet_sill",
    )
    shell.visual(
        Box((0.04, 0.04, 0.27)),
        origin=Origin(xyz=(-0.16, 0.16, 0.61)),
        material=housing_steel,
        name="left_jamb",
    )
    shell.visual(
        Box((0.04, 0.04, 0.27)),
        origin=Origin(xyz=(0.16, 0.16, 0.61)),
        material=housing_steel,
        name="right_jamb",
    )
    shell.visual(
        Box((0.36, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.16, 0.77)),
        material=housing_steel,
        name="outlet_header",
    )
    shell.inertial = Inertial.from_geometry(
        Box((0.52, 0.42, 0.83)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
    )

    band = model.part("hinge_reinforcement_band")
    band.visual(
        Box((0.40, 0.028, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=band_steel,
        name="front_band",
    )
    band.visual(
        Box((0.028, 0.058, 0.07)),
        origin=Origin(xyz=(-0.194, -0.015, 0.035)),
        material=band_steel,
        name="left_band_tab",
    )
    band.visual(
        Box((0.028, 0.058, 0.07)),
        origin=Origin(xyz=(0.194, -0.015, 0.035)),
        material=band_steel,
        name="right_band_tab",
    )
    band.inertial = Inertial.from_geometry(
        Box((0.40, 0.058, 0.07)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.006, 0.035)),
    )

    flap = model.part("weather_flap")
    flap.visual(
        Box((0.34, 0.012, 0.269)),
        origin=Origin(xyz=(0.0, 0.008, -0.1345)),
        material=flap_steel,
        name="flap_panel",
    )
    flap.visual(
        Box((0.34, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.011, -0.011)),
        material=flap_steel,
        name="flap_top_hem",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.34, 0.018, 0.269)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.010, -0.1345)),
    )

    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=shell,
        child=band,
        origin=Origin(xyz=(0.0, 0.194, 0.745)),
    )
    model.articulation(
        "band_to_flap",
        ArticulationType.REVOLUTE,
        parent=band,
        child=flap,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("main_shell")
    band = object_model.get_part("hinge_reinforcement_band")
    flap = object_model.get_part("weather_flap")
    flap_hinge = object_model.get_articulation("band_to_flap")

    ctx.expect_gap(
        band,
        shell,
        axis="y",
        positive_elem="front_band",
        negative_elem="outlet_header",
        max_gap=0.0002,
        max_penetration=0.0,
        name="reinforcement band seats flush to the outlet header plane",
    )
    ctx.expect_overlap(
        band,
        shell,
        axes="xz",
        elem_a="front_band",
        elem_b="outlet_header",
        min_overlap=0.035,
        name="reinforcement band spans the framed outlet header",
    )

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_gap(
            flap,
            shell,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="outlet_header",
            min_gap=0.004,
            max_gap=0.007,
            name="closed flap hangs just ahead of the outlet frame",
        )
        closed_panel_frame = ctx.part_element_world_aabb(flap, elem="flap_panel")
        left_jamb = ctx.part_element_world_aabb(shell, elem="left_jamb")
        right_jamb = ctx.part_element_world_aabb(shell, elem="right_jamb")
        sill = ctx.part_element_world_aabb(shell, elem="outlet_sill")
        header = ctx.part_element_world_aabb(shell, elem="outlet_header")
        ctx.check(
            "closed flap covers the framed outlet opening",
            closed_panel_frame is not None
            and left_jamb is not None
            and right_jamb is not None
            and sill is not None
            and header is not None
            and closed_panel_frame[0][0] <= left_jamb[1][0] - 0.005
            and closed_panel_frame[1][0] >= right_jamb[0][0] + 0.005
            and closed_panel_frame[0][2] <= sill[1][2] + 0.002
            and closed_panel_frame[1][2] >= header[0][2] - 0.002,
            details=(
                f"panel={closed_panel_frame}, left_jamb={left_jamb}, "
                f"right_jamb={right_jamb}, sill={sill}, header={header}"
            ),
        )

    closed_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 1.10}):
        open_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "weather flap opens outward and upward",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][1] > closed_panel[1][1] + 0.18
        and open_panel[0][2] > closed_panel[0][2] + 0.08,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
