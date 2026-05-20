from __future__ import annotations

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


ROOT_PIVOT = (0.0, 0.0, 0.320)
L0 = 0.300
L1 = 0.240
TAB_LENGTH = 0.145


def _add_tapered_plate(
    part,
    *,
    length: float,
    root_width: float,
    tip_width: float,
    thickness: float,
    z_offset: float,
    material,
    accent,
    name_prefix: str,
) -> None:
    """Add a flat link plate with round end bosses and visible bolt caps."""

    web_profile = [
        (0.010, -root_width * 0.40),
        (length - 0.010, -tip_width * 0.40),
        (length - 0.010, tip_width * 0.40),
        (0.010, root_width * 0.40),
    ]
    web_mesh = mesh_from_geometry(
        ExtrudeGeometry(web_profile, thickness, cap=True, center=True),
        f"{name_prefix}_web",
    )
    part.visual(
        web_mesh,
        origin=Origin(xyz=(0.0, 0.0, z_offset)),
        material=material,
        name="tapered_web",
    )

    root_radius = root_width * 0.58
    tip_radius = tip_width * 0.58
    part.visual(
        Cylinder(radius=root_radius, length=thickness),
        origin=Origin(xyz=(0.0, 0.0, z_offset)),
        material=material,
        name="proximal_boss",
    )
    part.visual(
        Cylinder(radius=tip_radius, length=thickness),
        origin=Origin(xyz=(length, 0.0, z_offset)),
        material=material,
        name="distal_boss",
    )

    cap_thickness = min(0.004, thickness * 0.22)
    cap_radius_root = root_radius * 0.46
    cap_radius_tip = tip_radius * 0.46
    cap_z = z_offset + thickness * 0.5 + cap_thickness * 0.5
    part.visual(
        Cylinder(radius=cap_radius_root, length=cap_thickness),
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
        material=accent,
        name="proximal_cap",
    )
    part.visual(
        Cylinder(radius=cap_radius_tip, length=cap_thickness),
        origin=Origin(xyz=(length, 0.0, cap_z)),
        material=accent,
        name="distal_cap",
    )


def _add_vertical_spacer(
    part,
    *,
    x: float,
    z_min: float,
    z_max: float,
    radius: float,
    material,
    name: str,
) -> None:
    length = z_max - z_min
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, 0.0, z_min + length * 0.5)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_revolute_chain")

    bridge_gray = model.material("powder_coated_bridge", color=(0.36, 0.39, 0.41, 1.0))
    dark_steel = model.material("dark_steel_pins", color=(0.08, 0.09, 0.10, 1.0))
    link_blue = model.material("anodized_proximal_link", color=(0.10, 0.23, 0.38, 1.0))
    link_teal = model.material("anodized_mid_link", color=(0.09, 0.37, 0.37, 1.0))
    tab_gold = model.material("bronze_end_tab", color=(0.72, 0.50, 0.24, 1.0))

    bridge = model.part("bridge_support")
    bridge.visual(
        Box((0.520, 0.280, 0.035)),
        origin=Origin(xyz=(0.020, 0.0, 0.0175)),
        material=bridge_gray,
        name="base_plate",
    )
    for y, name in ((-0.105, "post_0"), (0.105, "post_1")):
        bridge.visual(
            Box((0.055, 0.036, 0.310)),
            origin=Origin(xyz=(-0.055, y, 0.190)),
            material=bridge_gray,
            name=name,
        )
    bridge.visual(
        Box((0.280, 0.255, 0.050)),
        origin=Origin(xyz=(-0.055, 0.0, 0.365)),
        material=bridge_gray,
        name="top_bridge",
    )

    # Root clevis: two horizontal lugs sandwich the first link tab with clearance.
    bridge.visual(
        Box((0.185, 0.112, 0.012)),
        origin=Origin(xyz=(0.0125, 0.0, 0.348)),
        material=bridge_gray,
        name="root_upper_lug",
    )
    bridge.visual(
        Box((0.185, 0.112, 0.012)),
        origin=Origin(xyz=(0.0125, 0.0, 0.292)),
        material=bridge_gray,
        name="root_lower_lug",
    )
    bridge.visual(
        Box((0.028, 0.112, 0.068)),
        origin=Origin(xyz=(-0.066, 0.0, 0.320)),
        material=bridge_gray,
        name="root_clevis_web",
    )
    bridge.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.357)),
        material=dark_steel,
        name="root_top_cap",
    )
    bridge.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.283)),
        material=dark_steel,
        name="root_bottom_cap",
    )
    bridge.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=ROOT_PIVOT),
        material=dark_steel,
        name="root_pin",
    )

    proximal = model.part("proximal_link")
    _add_tapered_plate(
        proximal,
        length=L0,
        root_width=0.085,
        tip_width=0.072,
        thickness=0.030,
        z_offset=0.0,
        material=link_blue,
        accent=dark_steel,
        name_prefix="proximal",
    )
    # Shoulder spacer reaches from the upper face of the proximal link to the
    # underside of the smaller mid link, making the second joint visibly stacked.
    _add_vertical_spacer(
        proximal,
        x=L0,
        z_min=0.015,
        z_max=0.038,
        radius=0.016,
        material=dark_steel,
        name="mid_spacer",
    )

    mid = model.part("mid_link")
    _add_tapered_plate(
        mid,
        length=L1,
        root_width=0.064,
        tip_width=0.052,
        thickness=0.024,
        z_offset=0.050,
        material=link_teal,
        accent=dark_steel,
        name_prefix="mid",
    )
    _add_vertical_spacer(
        mid,
        x=L1,
        z_min=0.062,
        z_max=0.081,
        radius=0.012,
        material=dark_steel,
        name="distal_spacer",
    )

    end_tab = model.part("end_tab")
    _add_tapered_plate(
        end_tab,
        length=TAB_LENGTH,
        root_width=0.044,
        tip_width=0.034,
        thickness=0.018,
        z_offset=0.090,
        material=tab_gold,
        accent=dark_steel,
        name_prefix="end_tab",
    )
    end_tab.visual(
        Box((0.030, 0.040, 0.018)),
        origin=Origin(xyz=(TAB_LENGTH + 0.018, 0.0, 0.090)),
        material=tab_gold,
        name="compact_tip",
    )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=proximal,
        origin=Origin(xyz=ROOT_PIVOT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "mid_joint",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=mid,
        origin=Origin(xyz=(L0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.8, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "distal_joint",
        ArticulationType.REVOLUTE,
        parent=mid,
        child=end_tab,
        origin=Origin(xyz=(L1, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("root_joint"),
        object_model.get_articulation("mid_joint"),
        object_model.get_articulation("distal_joint"),
    ]
    ctx.check(
        "three serial revolute joints",
        len(joints) == 3
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints)
        and [joint.parent for joint in joints] == ["bridge_support", "proximal_link", "mid_link"]
        and [joint.child for joint in joints] == ["proximal_link", "mid_link", "end_tab"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in joints]}",
    )

    bridge = object_model.get_part("bridge_support")
    proximal = object_model.get_part("proximal_link")
    mid = object_model.get_part("mid_link")
    end_tab = object_model.get_part("end_tab")

    ctx.allow_overlap(
        bridge,
        proximal,
        elem_a="root_pin",
        elem_b="proximal_boss",
        reason="The bridge clevis pin intentionally passes through the first link pivot bore.",
    )

    ctx.expect_gap(
        bridge,
        proximal,
        axis="z",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="root_upper_lug",
        negative_elem="proximal_boss",
        name="root upper clevis clearance",
    )
    ctx.expect_gap(
        proximal,
        bridge,
        axis="z",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="proximal_boss",
        negative_elem="root_lower_lug",
        name="root lower clevis clearance",
    )
    ctx.expect_overlap(
        bridge,
        proximal,
        axes="xy",
        min_overlap=0.055,
        elem_a="root_upper_lug",
        elem_b="proximal_boss",
        name="root clevis surrounds first pivot",
    )
    ctx.expect_within(
        bridge,
        proximal,
        axes="xy",
        inner_elem="root_pin",
        outer_elem="proximal_boss",
        margin=0.001,
        name="root pin is centered in first link bore",
    )
    ctx.expect_overlap(
        bridge,
        proximal,
        axes="z",
        min_overlap=0.026,
        elem_a="root_pin",
        elem_b="proximal_boss",
        name="root pin engages the first link thickness",
    )
    ctx.expect_contact(
        proximal,
        mid,
        elem_a="mid_spacer",
        elem_b="proximal_boss",
        contact_tol=0.001,
        name="middle joint shoulder contacts upper link",
    )
    ctx.expect_contact(
        mid,
        end_tab,
        elem_a="distal_spacer",
        elem_b="proximal_boss",
        contact_tol=0.001,
        name="distal joint shoulder contacts end tab",
    )

    with ctx.pose({"root_joint": 0.55, "mid_joint": -0.75, "distal_joint": 0.65}):
        ctx.expect_gap(
            proximal,
            bridge,
            axis="z",
            min_gap=-0.002,
            positive_elem="proximal_boss",
            negative_elem="root_lower_lug",
            name="posed first link remains above lower clevis",
        )

    return ctx.report()


object_model = build_object_model()
