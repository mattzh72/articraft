from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scratch_dvs_two_channel_mixer")

    matte_black = Material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = Material("graphite_top", rgba=(0.075, 0.078, 0.085, 1.0))
    dark_panel = Material("dark_channel_panel", rgba=(0.045, 0.048, 0.055, 1.0))
    slot_black = Material("slot_black", rgba=(0.002, 0.002, 0.003, 1.0))
    brushed_metal = Material("brushed_metal", rgba=(0.62, 0.60, 0.56, 1.0))
    soft_gray = Material("soft_gray", rgba=(0.80, 0.78, 0.72, 1.0))
    white_print = Material("white_print", rgba=(0.88, 0.88, 0.82, 1.0))
    amber_led = Material("amber_led", rgba=(1.0, 0.52, 0.05, 1.0))
    green_led = Material("green_led", rgba=(0.1, 0.85, 0.24, 1.0))

    housing = model.part("housing")

    top_z = 0.058
    housing.visual(
        Box((0.62, 0.36, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=matte_black,
        name="wide_rect_body",
    )
    housing.visual(
        Box((0.586, 0.322, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=graphite,
        name="brushed_top_plate",
    )
    housing.visual(
        Box((0.020, 0.360, 0.020)),
        origin=Origin(xyz=(-0.300, 0.0, 0.062)),
        material=matte_black,
        name="side_rail_0",
    )
    housing.visual(
        Box((0.020, 0.360, 0.020)),
        origin=Origin(xyz=(0.300, 0.0, 0.062)),
        material=matte_black,
        name="side_rail_1",
    )
    housing.visual(
        Box((0.540, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.164, 0.060)),
        material=matte_black,
        name="rear_connector_lip",
    )
    housing.visual(
        Box((0.540, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.168, 0.060)),
        material=matte_black,
        name="front_scratch_lip",
    )

    for channel, x in enumerate((-0.145, 0.145)):
        housing.visual(
            Box((0.205, 0.278, 0.003)),
            origin=Origin(xyz=(x, 0.002, top_z + 0.0015)),
            material=dark_panel,
            name=f"channel_panel_{channel}",
        )
        housing.visual(
            Box((0.020, 0.128, 0.002)),
            origin=Origin(xyz=(x, -0.060, top_z + 0.003)),
            material=slot_black,
            name=f"channel_slot_{channel}",
        )
        housing.visual(
            Box((0.004, 0.138, 0.004)),
            origin=Origin(xyz=(x - 0.017, -0.060, top_z + 0.004)),
            material=brushed_metal,
            name=f"channel_slot_rail_{channel}_0",
        )
        housing.visual(
            Box((0.004, 0.138, 0.004)),
            origin=Origin(xyz=(x + 0.017, -0.060, top_z + 0.004)),
            material=brushed_metal,
            name=f"channel_slot_rail_{channel}_1",
        )
        for row, y in enumerate((0.115, 0.070, 0.025)):
            housing.visual(
                Cylinder(radius=0.018, length=0.002),
                origin=Origin(xyz=(x, y, top_z + 0.003)),
                material=brushed_metal,
                name=f"eq_bushing_{channel}_{row}",
            )
            housing.visual(
                Box((0.004, 0.016, 0.0015)),
                origin=Origin(xyz=(x, y + 0.028, top_z + 0.00375)),
                material=white_print,
                name=f"eq_tick_{channel}_{row}_0",
            )
            housing.visual(
                Box((0.014, 0.004, 0.0015)),
                origin=Origin(xyz=(x - 0.028, y, top_z + 0.00375)),
                material=white_print,
                name=f"eq_tick_{channel}_{row}_1",
            )
            housing.visual(
                Box((0.014, 0.004, 0.0015)),
                origin=Origin(xyz=(x + 0.028, y, top_z + 0.00375)),
                material=white_print,
                name=f"eq_tick_{channel}_{row}_2",
            )

        housing.visual(
            Box((0.054, 0.006, 0.0015)),
            origin=Origin(xyz=(x, -0.148, top_z + 0.00075)),
            material=white_print,
            name=f"channel_number_bar_{channel}",
        )

    housing.visual(
        Box((0.014, 0.286, 0.004)),
        origin=Origin(xyz=(0.0, 0.004, top_z + 0.003)),
        material=matte_black,
        name="center_divider",
    )
    housing.visual(
        Box((0.304, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, -0.138, top_z + 0.003)),
        material=slot_black,
        name="crossfader_slot",
    )
    housing.visual(
        Box((0.318, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.158, top_z + 0.002)),
        material=brushed_metal,
        name="crossfader_rail_0",
    )
    housing.visual(
        Box((0.318, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.118, top_z + 0.002)),
        material=brushed_metal,
        name="crossfader_rail_1",
    )

    for i, y in enumerate((0.120, 0.104, 0.088, 0.072, 0.056, 0.040)):
        housing.visual(
            Cylinder(radius=0.0045, length=0.0015),
            origin=Origin(xyz=(0.0, y, top_z + 0.0045)),
            material=green_led if i < 4 else amber_led,
            name=f"level_led_{i}",
        )

    fader_limits = MotionLimits(effort=10.0, velocity=0.35, lower=-0.040, upper=0.040)
    for channel, x in enumerate((-0.145, 0.145)):
        fader = model.part(f"channel_fader_{channel}")
        fader.visual(
            Box((0.012, 0.030, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=brushed_metal,
            name="fader_stem",
        )
        fader.visual(
            Box((0.070, 0.026, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=soft_gray,
            name="fader_cap",
        )
        fader.visual(
            Box((0.008, 0.024, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0295)),
            material=matte_black,
            name="fader_grip_line",
        )
        model.articulation(
            f"housing_to_channel_fader_{channel}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, -0.060, top_z + 0.004)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=fader_limits,
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.041,
            0.024,
            body_style="faceted",
            base_diameter=0.043,
            top_diameter=0.034,
            edge_radius=0.001,
            skirt=KnobSkirt(0.047, 0.004, flare=0.04, chamfer=0.001),
            grip=KnobGrip(style="ribbed", count=18, depth=0.0009, width=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "eq_knob_faceted",
    )
    for channel, x in enumerate((-0.145, 0.145)):
        for row, y in enumerate((0.115, 0.070, 0.025)):
            knob = model.part(f"eq_knob_{channel}_{row}")
            knob.visual(knob_mesh, origin=Origin(), material=matte_black, name="eq_knob_cap")
            knob.visual(
                Box((0.004, 0.024, 0.0015)),
                origin=Origin(xyz=(0.0, 0.007, 0.0245)),
                material=white_print,
                name="knob_indicator_line",
            )
            model.articulation(
                f"housing_to_eq_knob_{channel}_{row}",
                ArticulationType.CONTINUOUS,
                parent=housing,
                child=knob,
                origin=Origin(xyz=(x, y, top_z + 0.004)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=1.4, velocity=8.0),
            )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.032, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_metal,
        name="crossfader_stem",
    )
    crossfader.visual(
        Box((0.034, 0.048, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=soft_gray,
        name="crossfader_cap",
    )
    crossfader.visual(
        Box((0.026, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=matte_black,
        name="crossfader_grip_line",
    )
    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.138, top_z + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.5, lower=-0.118, upper=0.118),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    channel_faders = [object_model.get_part(f"channel_fader_{i}") for i in range(2)]
    channel_joints = [object_model.get_articulation(f"housing_to_channel_fader_{i}") for i in range(2)]
    crossfader = object_model.get_part("crossfader")
    cross_joint = object_model.get_articulation("housing_to_crossfader")
    eq_joints = [
        object_model.get_articulation(f"housing_to_eq_knob_{channel}_{row}")
        for channel in range(2)
        for row in range(3)
    ]

    ctx.check(
        "two channel faders slide in vertical slots",
        len(channel_faders) == 2
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in channel_joints)
        and all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in channel_joints),
        details=f"joints={[j.name for j in channel_joints]}",
    )
    ctx.check(
        "six eq knobs rotate continuously",
        len(eq_joints) == 6
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in eq_joints)
        and all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in eq_joints),
        details=f"joints={[j.name for j in eq_joints]}",
    )
    ctx.check(
        "crossfader slides horizontally",
        cross_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(cross_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={cross_joint.axis}",
    )

    for channel, fader in enumerate(channel_faders):
        ctx.expect_gap(
            fader,
            housing,
            axis="z",
            positive_elem="fader_stem",
            negative_elem=f"channel_slot_{channel}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"channel fader {channel} stem sits on slot",
        )
    ctx.expect_gap(
        crossfader,
        housing,
        axis="z",
        positive_elem="crossfader_stem",
        negative_elem="crossfader_slot",
        max_gap=0.001,
        max_penetration=0.0,
        name="crossfader stem sits on slot",
    )

    def _aabb(part_name: str, elem: str):
        return ctx.part_element_world_aabb(object_model.get_part(part_name), elem=elem)

    def _within_xy(inner, outer, slack: float = 0.001) -> bool:
        return (
            inner is not None
            and outer is not None
            and inner[0][0] >= outer[0][0] - slack
            and inner[1][0] <= outer[1][0] + slack
            and inner[0][1] >= outer[0][1] - slack
            and inner[1][1] <= outer[1][1] + slack
        )

    for channel, joint in enumerate(channel_joints):
        for limit_name, value in (("lower", joint.motion_limits.lower), ("upper", joint.motion_limits.upper)):
            with ctx.pose({joint: value}):
                ctx.check(
                    f"channel fader {channel} stem remains within {limit_name} slot end",
                    _within_xy(_aabb(f"channel_fader_{channel}", "fader_stem"), _aabb("housing", f"channel_slot_{channel}")),
                    details=f"q={value}",
                )

    with ctx.pose({cross_joint: cross_joint.motion_limits.lower}):
        ctx.check(
            "crossfader stem remains within left slot end",
            _within_xy(_aabb("crossfader", "crossfader_stem"), _aabb("housing", "crossfader_slot")),
            details=f"q={cross_joint.motion_limits.lower}",
        )
    with ctx.pose({cross_joint: cross_joint.motion_limits.upper}):
        ctx.check(
            "crossfader stem remains within right slot end",
            _within_xy(_aabb("crossfader", "crossfader_stem"), _aabb("housing", "crossfader_slot")),
            details=f"q={cross_joint.motion_limits.upper}",
        )

    fader_rest = ctx.part_world_position(channel_faders[0])
    with ctx.pose({channel_joints[0]: channel_joints[0].motion_limits.upper}):
        fader_upper = ctx.part_world_position(channel_faders[0])
    cross_rest = ctx.part_world_position(crossfader)
    with ctx.pose({cross_joint: cross_joint.motion_limits.upper}):
        cross_upper = ctx.part_world_position(crossfader)
    ctx.check(
        "channel fader upper pose moves along slot",
        fader_rest is not None and fader_upper is not None and fader_upper[1] > fader_rest[1] + 0.035,
        details=f"rest={fader_rest}, upper={fader_upper}",
    )
    ctx.check(
        "crossfader right pose moves horizontally",
        cross_rest is not None and cross_upper is not None and cross_upper[0] > cross_rest[0] + 0.10,
        details=f"rest={cross_rest}, upper={cross_upper}",
    )

    return ctx.report()


object_model = build_object_model()
